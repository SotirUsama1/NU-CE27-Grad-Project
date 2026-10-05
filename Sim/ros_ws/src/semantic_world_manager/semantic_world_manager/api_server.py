from fastapi import FastAPI, HTTPException, Query

from .schemas import HealthStatus, LocatedEntity, SceneSummary


def create_app(memory_core):
    app = FastAPI(title="Semantic World Manager", docs_url=None, redoc_url=None)

    @app.get("/health", response_model=HealthStatus)
    def health():
        return memory_core.health()

    @app.get("/api/scene/summary", response_model=SceneSummary)
    def summary():
        return memory_core.summary()

    @app.get("/api/scene/locate/{object_id}", response_model=LocatedEntity)
    def locate(object_id: str):
        entity = memory_core.get_entity(object_id)
        if entity is None:
            raise HTTPException(status_code=404, detail="Object not found")
        entity["loc"] = [round(value, 2) for value in entity["loc"]]
        entity["yaw_deg"] = round(entity["yaw_deg"], 2)
        entity["size"] = [round(value, 2) for value in entity["size"]]
        return entity

    @app.get("/api/scene/spatial")
    def spatial(target: str, radius: float = Query(ge=0.0)):
        matches = memory_core.spatial(target, radius)
        if matches is None:
            raise HTTPException(status_code=404, detail="Target object not found")
        return matches

    @app.get("/api/scene/objects")
    def objects(category: str = None):
        return memory_core.objects(category)

    return app
