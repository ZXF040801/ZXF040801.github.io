import logging
import requests
from pydub import AudioSegment
from pydub.playback import play
from io import BytesIO
from urllib.parse import quote

API_KEY = "ECDuC92Jkny3X4tyccT6wqII"
SECRET_KEY = "GX9Rxn2RTPn5OVy4xAqxmadBdppJP6yB"

spd = 8 # 语速 0-15
pit = 5 # 音调 0-15
vol = 5 # 音量 0-15
per = 0 # 音库 度小宇=1，度小美=0，度逍遥（基础）=3，度丫丫=4 度逍遥（精品）=5003，度小鹿=5118，度博文=106，度小童=110，度小萌=111，度米朵=103，度小娇=5


def TTS(msg):
    logger.info(msg)
    url = "https://tsn.baidu.com/text2audio"
    encoded_str = quote(msg, encoding="utf-8")

    payload = f'tex={encoded_str}&tok={access_token}&cuid=1HNHg2B33TItBtlBq4FamLRAoZ2jxRNr&ctp=1&lan=zh&spd={spd}&pit={pit}&vol={vol}&per={per}&aue=6'
    headers = {
        'Content-Type': 'application/x-www-form-urlencoded',
        'Accept': '*/*'
    }

    response = requests.request("POST", url, headers=headers, data=payload.encode("utf-8"))

    my_play(response.content)

def my_play(binary_data):
    audio = AudioSegment.from_wav(BytesIO(binary_data))
    play(audio)

def get_access_token():
    """
    使用 AK，SK 生成鉴权签名（Access Token）
    :return: access_token，或是None(如果错误)
    """
    url = "https://aip.baidubce.com/oauth/2.0/token"
    params = {"grant_type": "client_credentials", "client_id": API_KEY, "client_secret": SECRET_KEY}
    return str(requests.post(url, params=params).json().get("access_token"))


logger = logging.getLogger("voice_interface_node.TTS")
access_token = get_access_token()

if __name__ == "__main__":
    TTS("我是你的语音助手")
