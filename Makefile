# Convenience targets so the base images are always built before the modules.
# `docker compose` can't build them itself (they're FROM dependencies, not
# services), so these wrap the two steps together.

.PHONY: base base-slim base-cuda build up up-d down logs

# No module needs the CUDA base today, so `base` builds only the slim one.
# Build the CUDA base explicitly (make base-cuda) when a GPU module needs it.
base: base-slim            ## Build the base image every module currently uses

base-slim:                 ## Build the slim CPU base (all modules today)
	docker build -t rammp-base:humble docker/base

base-cuda:                 ## Build the CUDA base (for a future GPU module)
	docker build -t rammp-base-cuda:humble docker/base-cuda

build: base                ## Build all module images (base first)
	docker compose build

up: base                   ## Build if needed and run in the foreground
	docker compose up --build

up-d: base                 ## Same, detached
	docker compose up --build -d

down:                      ## Stop and remove containers
	docker compose down

logs:                      ## Follow logs
	docker compose logs -f
