# Hosting the dashboard on AWS

The dashboard is a single small Node.js server packaged as a Docker image, so it
runs the same on your PC and on AWS. You only need to change environment
variables; no code changes.

> **Note:** AWS App Runner is no longer open to new customers. AWS recommends
> **Amazon ECS Express Mode** instead, which is what Option A uses.

## What you need on the AWS side

- An AWS account and the AWS CLI installed (`aws configure`)
- Docker installed on your PC
- A region, e.g. `eu-central-1` (Frankfurt) or `me-south-1` (Bahrain)

## Step 1: Store the GitHub token in Secrets Manager

Never bake the token into the image.

```bash
aws secretsmanager create-secret \
  --name github-dashboard/token \
  --secret-string "ghp_your_token_here" \
  --region eu-central-1
```

Note the secret ARN it prints.

## Step 2: Build the image and push it to Amazon ECR

```bash
REGION=eu-central-1
ACCOUNT=$(aws sts get-caller-identity --query Account --output text)
REPO=github-dashboard

aws ecr create-repository --repository-name $REPO --region $REGION
aws ecr get-login-password --region $REGION | docker login --username AWS --password-stdin $ACCOUNT.dkr.ecr.$REGION.amazonaws.com

# On Apple Silicon / ARM machines add: --platform linux/amd64
docker build -t $REPO .
docker tag $REPO:latest $ACCOUNT.dkr.ecr.$REGION.amazonaws.com/$REPO:latest
docker push $ACCOUNT.dkr.ecr.$REGION.amazonaws.com/$REPO:latest
```

## Option A (recommended): Amazon ECS Express Mode

1. Open the ECS console and choose **Express mode** in the left menu.
2. **Image URI:** choose *Browse ECR images* and pick `github-dashboard:latest`.
3. **IAM roles:** for the task execution role and infrastructure role choose
   *Create new role* if you don't have them yet.
4. Open **Additional configurations**:
   - **Container port:** `8080`
   - **Health check path:** `/health`
   - **Environment variables:**

     | Key | Value type | Value |
     |---|---|---|
     | `GITHUB_REPOS` | Environment variable | `my-org/backend,my-org/frontend` |
     | `DASHBOARD_PASSWORD` | Environment variable (or Secret) | a strong password |
     | `DEPLOY_TARGET` | Environment variable | `ecs` |
     | `AWS_REGION` | Environment variable | `eu-central-1` |
     | `GITHUB_TOKEN` | **Secret** | the secret ARN from step 1 |

5. Choose **Create**. When it's done, ECS shows an **Application URL**. That's
   your dashboard link. Add it as `PUBLIC_URL` so it appears in the AWS panel.

Because `GITHUB_TOKEN` is passed as a *Secret*, the **task execution role**
needs permission to read it. Add this inline policy to that role:

```json
{
  "Version": "2012-10-17",
  "Statement": [{
    "Effect": "Allow",
    "Action": "secretsmanager:GetSecretValue",
    "Resource": "arn:aws:secretsmanager:REGION:ACCOUNT:secret:github-dashboard/token-*"
  }]
}
```

To update after code changes: rebuild, push the image again, and deploy a new
revision of the service from the ECS console.

## Option B (cheapest/simplest): a single EC2 instance

1. Launch a small instance (e.g. `t3.micro`, Amazon Linux 2023). In the security
   group allow inbound TCP `80` (and `22` for SSH from your IP only).
2. SSH in and install Docker:
   ```bash
   sudo dnf install -y docker && sudo systemctl enable --now docker
   ```
3. Copy the project (or pull the image from ECR), create a `.env` file on the
   server, then run:
   ```bash
   sudo docker build -t github-dashboard .
   sudo docker run -d --restart unless-stopped --env-file .env \
     -e PORT=8080 -e DEPLOY_TARGET=ec2 -p 80:8080 github-dashboard
   ```
4. Open `http://<instance-public-ip>`. For HTTPS and a nice domain, put it
   behind an Application Load Balancer or CloudFront with an ACM certificate.

## Alternative to "Secret" env vars: `AWS_SECRET_NAME`

Instead of injecting `GITHUB_TOKEN` as a secret, you can set
`AWS_SECRET_NAME=github-dashboard/token` and the app reads the secret itself at
startup (see `src/aws.js`). In that case the **task role** (Option A) or the
**instance profile** (Option B) needs `secretsmanager:GetSecretValue`. The
secret may also be JSON with several keys, e.g.
`{"GITHUB_TOKEN":"...","DASHBOARD_PASSWORD":"..."}`.

## Security checklist before sharing the link

- [ ] `DASHBOARD_PASSWORD` is set (the dashboard shows private repo activity)
- [ ] GitHub token is read-only and limited to the repos you list
- [ ] Token lives in Secrets Manager, not in the image or in git
- [ ] HTTPS is enabled (ECS Express Mode gives you this through its load balancer)
