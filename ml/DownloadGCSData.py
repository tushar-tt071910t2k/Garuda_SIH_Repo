# DownloadGCSData.py
# Downloads all files from gs://idd1 bucket to local ml/data/ folder
# Run from MATLAB: pyrunfile('ml/DownloadGCSData.py')
#
# BEFORE RUNNING:
#   Place your service account JSON key at:  e:\sih_matlab\ml\gcs_key.json

import os, sys, json
from pathlib import Path

SERVICE_ACCOUNT_KEY = r"E:\sih_matlab\ml\gcs_key.json"
BUCKET_NAME         = "idd1"
LOCAL_DIR           = r"E:\sih_matlab\ml\data"

# Verify key file exists
if not os.path.exists(SERVICE_ACCOUNT_KEY):
    print(f"ERROR: Service account key not found at: {SERVICE_ACCOUNT_KEY}")
    print("Please place your GCS key JSON file there and re-run.")
    sys.exit(1)

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = SERVICE_ACCOUNT_KEY

try:
    from google.cloud import storage
except ImportError:
    print("ERROR: google-cloud-storage not installed.")
    print("Run: pip install google-cloud-storage")
    sys.exit(1)

print(f"Connecting to gs://{BUCKET_NAME} ...")
client = storage.Client()
bucket = client.bucket(BUCKET_NAME)

# List all blobs
blobs = list(bucket.list_blobs())
print(f"Found {len(blobs)} files in bucket.")

downloaded = 0
skipped    = 0

for blob in blobs:
    local_path = os.path.join(LOCAL_DIR, blob.name)
    local_dir  = os.path.dirname(local_path)
    os.makedirs(local_dir, exist_ok=True)

    # Skip if already downloaded and same size
    if os.path.exists(local_path) and os.path.getsize(local_path) == blob.size:
        skipped += 1
        continue

    print(f"  Downloading: {blob.name}  ({blob.size/1024:.1f} KB)")
    blob.download_to_filename(local_path)
    downloaded += 1

print(f"\nDone! Downloaded: {downloaded}  Skipped (cached): {skipped}")
print(f"Data saved to: {LOCAL_DIR}")

# Write a manifest so MATLAB knows what's available
manifest = {
    "bucket": BUCKET_NAME,
    "total_files": len(blobs),
    "files": [b.name for b in blobs]
}
with open(os.path.join(LOCAL_DIR, "manifest.json"), "w") as f:
    json.dump(manifest, f, indent=2)
print("Manifest written: ml/data/manifest.json")
