// ─────────────────────────────────────────────────────────────────────────────
//  AWS CONNECTION SECTION
//  Everything AWS-specific lives here. Locally none of this is needed: the app
//  reads GITHUB_TOKEN from .env. On AWS you can instead keep the token in
//  AWS Secrets Manager by setting AWS_SECRET_NAME (and AWS_REGION).
// ─────────────────────────────────────────────────────────────────────────────

let tokenSource = 'none';

export function setTokenSource(source) {
  tokenSource = source;
}

// Pull secrets (e.g. GITHUB_TOKEN) from AWS Secrets Manager at startup.
// The secret can be plain text (just the token) or JSON like
// {"GITHUB_TOKEN":"ghp_...","DASHBOARD_PASSWORD":"..."}.
export async function loadSecretsFromAws() {
  const secretName = process.env.AWS_SECRET_NAME;
  if (!secretName) return { loaded: false, reason: 'AWS_SECRET_NAME not set' };

  let sdk;
  try {
    sdk = await import('@aws-sdk/client-secrets-manager');
  } catch {
    const msg = 'AWS_SECRET_NAME is set but @aws-sdk/client-secrets-manager is not installed. Run: npm install';
    console.warn(`[aws] ${msg}`);
    return { loaded: false, reason: msg };
  }

  try {
    const client = new sdk.SecretsManagerClient({ region: process.env.AWS_REGION });
    const out = await client.send(new sdk.GetSecretValueCommand({ SecretId: secretName }));
    const value = out.SecretString || '';
    let parsed = null;
    try { parsed = JSON.parse(value); } catch { /* plain-text secret */ }

    if (parsed && typeof parsed === 'object') {
      for (const [k, v] of Object.entries(parsed)) process.env[k] = String(v);
    } else if (value) {
      process.env.GITHUB_TOKEN = value.trim();
    }
    tokenSource = 'aws-secrets-manager';
    console.log(`[aws] Loaded secrets from Secrets Manager (${secretName})`);
    return { loaded: true };
  } catch (err) {
    console.warn(`[aws] Could not read secret "${secretName}": ${err.message}`);
    return { loaded: false, reason: err.message };
  }
}

// Shown in the "AWS connection" panel of the dashboard.
export function awsStatus() {
  const runningOnAws = Boolean(
    process.env.AWS_EXECUTION_ENV ||
    process.env.ECS_CONTAINER_METADATA_URI_V4 ||
    process.env.AWS_LAMBDA_FUNCTION_NAME ||
    (process.env.DEPLOY_TARGET && process.env.DEPLOY_TARGET !== 'local')
  );
  return {
    connected: runningOnAws,
    deployTarget: process.env.DEPLOY_TARGET || 'local',
    region: process.env.AWS_REGION || null,
    secretName: process.env.AWS_SECRET_NAME || null,
    tokenSource,
    publicUrl: process.env.PUBLIC_URL || null,
  };
}
