"""Application configuration via ``pydantic-settings``.

Settings are populated from environment variables, falling back to a local
``.env`` file when present, and finally to the defaults below. See
``backend/.env.example`` for the documented local development values.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8")

    DATABASE_URL: str = "postgresql://cyberscape:cyberscape@localhost:5432/cyberscape"
    REDIS_URL: str = "redis://localhost:6379/0"
    ROSBRIDGE_WS_URL: str = "ws://localhost:9090"
    TOPO_MAP_PATH: str = "data/topo_graph.json"
    HEARTBEAT_TIMEOUT_S: float = 3.0
    HEARTBEAT_QUARANTINE_S: float = 10.0
    SEGMENT_LOCK_TTL_S: float = 5.0
    DISPATCH_SWEEP_INTERVAL_S: float = 7.0
    AUTH_API_KEY: str = "cyberscape-secret-key"

    # Agent worker (Phase 6): LangGraph LLM settings. An empty OPENAI_API_KEY
    # keeps the worker fully deterministic (regex mission parsing + heuristic
    # failure triage) so no OpenAI key is needed in CI or offline development.
    OPENAI_API_KEY: str = ""
    OPENAI_MODEL: str = "gpt-4o-mini"
    PROMPT_MAX_BOXES_PER_ZONE: int = 6
    SUPERVISOR_MAX_RETRIES: int = 2


settings = Settings()