import base64
import logging
import urllib
import requests
import json
import os

API_KEY = "ECDuC92Jkny3X4tyccT6wqII"
SECRET_KEY = "GX9Rxn2RTPn5OVy4xAqxmadBdppJP6yB"

logger = logging.getLogger("voice_interface_node.ASR")

def ASR(filename,format,rate):
    url = "https://vop.baidu.com/server_api"

    speech = get_file_content_as_base64(filename)
    file_size = os.path.getsize(filename)
    payload = json.dumps({
        "format": format,
        "rate": rate,
        "channel": 1,
        "cuid": "fuRquo7aQuAxEFtnAtkOp8i6DmzqrhWI",
        "token": get_access_token(),
        "speech": speech,
        "len": file_size,
        "dev_pid": 1737
    }, ensure_ascii=False)
    headers = {
        'Content-Type': 'application/json',
        'Accept': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload.encode("utf-8"))

    if response.status_code != 200:
        logger.error(f"ASR API接口异常")
        return None
    data = json.loads(response.text)
    if data["err_no"] != 0:
        logger.error(f"ASR 请求异常：{data['err_no']} {data['err_msg']}")
        return None

    logger.info(data["result"][0])
    return data["result"][0]


def ASR2(filename,format,rate):
    url = "https://vop.baidu.com/pro_api"

    speech = get_file_content_as_base64(filename)
    file_size = os.path.getsize(filename)
    payload = json.dumps({
        "format": format,
        "rate": rate,
        "channel": 1,
        "cuid": "fuRquo7aQuAxEFtnAtkOp8i6DmzqrhWI",
        "dev_pid": 80001,
        "token": access_token,
        "speech": speech,
        "len": file_size
    }, ensure_ascii=False)
    headers = {
        'Content-Type': 'application/json',
        'Accept': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload.encode("utf-8"))

    if response.status_code != 200:
        logger.error(f"ASR API接口异常")
        return None
    data = json.loads(response.text)
    if data["err_no"] != 0:
        logger.error(f"ASR 请求异常：{data['err_no']} {data['err_msg']}")
        return None

    logger.info(data["result"][0])
    return data["result"][0]


def get_file_content_as_base64(path, urlencoded=False):
    """
    获取文件base64编码
    :param path: 文件路径
    :param urlencoded: 是否对结果进行urlencoded
    :return: base64编码信息
    """
    with open(path, "rb") as f:
        content = base64.b64encode(f.read()).decode("utf8")
        if urlencoded:
            content = urllib.parse.quote_plus(content)
    return content


def get_access_token():
    """
    使用 AK，SK 生成鉴权签名（Access Token）
    :return: access_token，或是None(如果错误)
    """
    url = "https://aip.baidubce.com/oauth/2.0/token"
    params = {"grant_type": "client_credentials", "client_id": API_KEY, "client_secret": SECRET_KEY}
    return str(requests.post(url, params=params).json().get("access_token"))

access_token = get_access_token()

if __name__ == '__main__':
    d = ASR2("123.m4a", "m4a", 16000)
    print(d)
