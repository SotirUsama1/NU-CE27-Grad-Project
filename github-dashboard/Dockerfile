# Container image for hosting the dashboard on AWS (ECS Express Mode, ECS, EC2, ...)
FROM node:22-alpine

WORKDIR /app
ENV NODE_ENV=production \
    PORT=8080

COPY package.json ./
# Installs the optional AWS SDK so AWS_SECRET_NAME works inside AWS
RUN npm install --omit=dev --no-audit --no-fund

COPY server.js ./
COPY src ./src
COPY public ./public

USER node
EXPOSE 8080
HEALTHCHECK --interval=30s --timeout=5s CMD wget -qO- http://localhost:8080/health || exit 1
CMD ["node", "server.js"]
