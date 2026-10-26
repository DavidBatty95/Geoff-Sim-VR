FROM python:3.11-slim-bookworm@sha256:f06d01d4ba3f8ad5af45a3ec87df19c1d317d8e8961edc0fabda61811af6f7cf
RUN apt-get update && apt-get install -y --no-install-recommends \
    libosmesa6 libgl1 libegl1 libglib2.0-0 openssl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY requirements.txt constraints.txt ./
RUN pip install --no-cache-dir -c constraints.txt -r requirements.txt && pip check
ENV MUJOCO_GL=osmesa PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH=/app:/app/vendor/televuer OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
COPY . .
RUN python -m dougie_vr.model
EXPOSE 8012 8080
HEALTHCHECK --interval=15s --timeout=3s --start-period=40s \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8080/health',timeout=2)"
CMD ["python", "-m", "dougie_vr.app"]
