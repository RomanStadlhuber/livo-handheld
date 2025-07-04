# Webinterface for Handheld LiDAR-Inertial-Camera Recorder

With this interface you can create and manage recordings from a remotely connected client (e.g. your smartphone).
It is invoked automatically in `entrypoint.sh` which is used in the `deploy` stage of the Docker build process (started by `scripts/docker/prod.sh`).

## Quick Start

For local development, inside the container from `scripts/docker/run_image.sh`, navigate to the `webinterface/` folder and install the dependencies with

```bash
pip install -r requirements.txt
```

then run the app using

```bash
python3 app.py
```

## Production

When hosting a production server with `gunicorn`, always **only use a single worker**.

Due to the way in which state is managed in globals, internal consistency is lost when forking a new worker process.
This will in turn lead to unexpected behavior like nodes not starting or shutting down, recordings not finishing and duplicate recordings.

```bash
gunicorn -w 1 -b 0.0.0.0:8000 app:app
```

### Third Party Acknowledgement

This app was created using [bootstrap-flask](https://github.com/helloflask/bootstrap-flask) (visit the [documentation](https://bootstrap-flask.readthedocs.io/en/stable/basic/)).
The templates are largely copied and modified based on the provided examples!
