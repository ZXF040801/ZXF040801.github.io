import json
import logging

import requests
from openai import OpenAI

logger = logging.getLogger("voice_interface_node.NLP")

SYS_ROLE = ("你是一个安装在乒乓球拾取机器人上的语音助手，用于对机器人进行控制。"
            "用户输入的信息来自于语音识别，故用户输入的信息可能有误差。用户可以通过与你对话从而对机器人进行以下操作："
            "1：暂停任务：机器人处在运行阶段，用户希望机器人暂时暂停任务；"
            "2：继续任务：机器人处于暂停任务阶段，用户希望机器人继续执行任务；"
            "3：开始拾取：机器人处于待机状态，用户希望机器人开始执行乒乓球拾取任务；"
            "4：返回基站：机器人处于任务状态或暂停任务状态，用户希望机器人立即返回基站。若机器人已经在返航状态，则无需重复执行，若机器人在地图初始化状态，则禁止返航。"
            "5：取消任务：机器人处于非待机状态，用户希望机器人取消正在进行的任务，立刻进入待机状态。"
            "6：开始建图：机器人处于待机状态，或者地图未初始化状态，用户希望机器人执行SLAM建图工作。"
            "用户给你的信息将包括以下内容："
            "msg：用户所说的话；status：机器人的状态，包括如下可能的值： 0->待机，1->执行任务中，2->任务暂停中，3->返航中，4->地图初始化中，5->地图未初始化"
            "请注意，你的返回值必须符合json格式，输出需要直接可以被json解析。若经过你的分析，用户希望你进行上述某个操作，"
            "则令‘do’键为需要进行的操作的序号，令‘talk’为一句简单的回应；若用户的要求与上述逻辑冲突，则令‘error’键为冲突信息；"
            "若用户只是与你闲聊，则令‘talk’为你的回复。")

NAME = "小机器人"
AWAKE_ROLE = (f"请你充当一个语音唤醒检测助手，你的名字叫做{NAME}，用户给你的输入来源于语音识别模块，故文字可能有所误差，若拼音相同，也可认定为相同唤醒词。"
              f"你的输出应严格遵守json格式，当用户尝试唤醒你时，你需要在字典的‘wake’字段输出true，否则输出false。")

def gemini_api(msg:dict):
    try:
        client = OpenAI(
            api_key="AIzaSyBO4r9gjzppCSFxygCje0_yhTrqu-1fF40",
            base_url="https://gemini.200431.xyz"
        )

        response = client.chat.completions.create(
            model="gemini-2.0-flash",
            n=1,
            messages=[
                {"role": "system", "content": SYS_ROLE},
                {
                    "role": "user",
                    "content": json.dumps(msg)
                }
            ]
        )

        # 获取消息内容
        if response and response.choices and len(response.choices) > 0 and response.choices[0].message:
            re = response.choices[0].message.content
            re = re.replace('```json', '')
            re = re.replace('```', '')
            result = json.loads(re)
            return result
        else:
            logger.error("API响应格式异常")
            return {"error":"抱歉，我现在无法正常回答"}

    except Exception as e:
        logger.exception(f"调用Gemini API时发生错误")
        return {"error":"抱歉，系统出现了一些问题"}


def nlp(dct:dict) -> dict:
    re = gemini_api(dct)
    logger.debug(re)
    return re

def siliconflow_api(msg):
    import requests

    try:
        url = "https://api.siliconflow.cn/v1/chat/completions"

        payload = {
            "model": "Qwen/Qwen2-7B-Instruct",
            "messages": [
                {"role": "system", "content": AWAKE_ROLE},
                {
                    "role": "user",
                    "content": msg
                }
            ],
        }
        headers = {
            "Authorization": "Bearer sk-pdnswyjcptwituvetfujxcfkgwrrbcxijizaitrmuhbhtzuh",
            "Content-Type": "application/json"
        }

        response = requests.request("POST", url, json=payload, headers=headers)
        result = json.loads(response.text)
        # 获取消息内容
        if result and result['choices'] and len(result['choices']) > 0 and result['choices'][0]['message']:
            re = result['choices'][0]['message']['content']
            re = re.replace('```json', '')
            re = re.replace('```', '')
            result = json.loads(re)
            return result
        else:
            logger.error("API响应格式异常")
            return {"error": "抱歉，我现在无法正常回答"}

    except Exception as e:
        logger.exception(f"调用硅基流动API时发生错误")
        return {"error": "抱歉，系统出现了一些问题"}

def awake(msg) -> dict:
    re = siliconflow_api(msg)
    logger.debug(re)
    return re

if __name__ == '__main__':
    dct = {
         "msg": "帮我清理",
         "status": 0
    }
    result = nlp(dct)
    print(result)

