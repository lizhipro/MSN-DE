import os
import requests

# ========== 配置 ==========
ACCESS_TOKEN = "your_api_token_here"   # 替换成你的 Zenodo API Token
RECORD_ID = "1234567"                  # 替换成你的 record id
FILES_DIR = "./files_to_upload"        # 要上传的文件夹

# Zenodo API
ZENODO_URL = f"https://zenodo.org/api/deposit/depositions/{RECORD_ID}"

# 上传函数
def upload_file(file_path):
    url = f"{ZENODO_URL}/files"
    headers = {"Authorization": f"Bearer {ACCESS_TOKEN}"}
    filename = os.path.basename(file_path)

    print(f"⬆️ Uploading {filename} ...")
    with open(file_path, "rb") as fp:
        r = requests.post(
            url,
            headers=headers,
            data={"name": filename},
            files={"file": fp}
        )
    if r.status_code == 201:
        print(f"✅ {filename} uploaded successfully.")
    else:
        print(f"❌ Failed to upload {filename}: {r.status_code}, {r.text}")

def main():
    # 遍历目录下所有文件
    for fname in sorted(os.listdir(FILES_DIR)):
        fpath = os.path.join(FILES_DIR, fname)
        if os.path.isfile(fpath):
            upload_file(fpath)

if __name__ == "__main__":
    main()
