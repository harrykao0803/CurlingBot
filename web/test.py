import requests
import json

BASE_URL = 'https://tomcheng.me'

# Set and get emotion example
def set_emotion(emotion):
    response = requests.post(f"{BASE_URL}/set_emotion", data={"emotion": emotion})
    if response.status_code == 200:
        print(f"Set Emotion Response: {response.json()}")
    else:
        print(f"Failed to set emotion: {response.status_code}, {response.text}")


def get_emotion():
    response = requests.get(f"{BASE_URL}/get_emotion")
    if response.status_code == 200:
        print(f"Get Emotion Response: {response.json()}")
    else:
        print(f"Failed to get emotion: {response.status_code}, {response.text}")


def set_state():
    response = requests.post(f"{BASE_URL}/set_state", data={"state": 'menu', "round": 1})
    if response.status_code == 200:
        print(f"Set Emotion Response: {response.json()}")
    else:
        print(f"Failed to set emotion: {response.status_code}, {response.text}")


def get_state():
    response = requests.get(f"{BASE_URL}/get_state")
    if response.status_code == 200:
        print(f"Get Emotion Response: {response.json()}")
    else:
        print(f"Failed to get emotion: {response.status_code}, {response.text}")


def set_result(res):
    response = requests.post(f"{BASE_URL}/set_result", data={'result_pos': res})
    if response.status_code == 200:
        print(f"Set Emotion Response: {response.json()}")
    else:
        print(f"Failed to set emotion: {response.status_code}, {response.text}")


def get_result():
    response = requests.get(f"{BASE_URL}/get_result")
    return response.json()



def progress():
    response = requests.post(f"{BASE_URL}/progress", data={})
    if response.status_code == 200:
        print(f"Set Emotion Response: {response.json()}")
    else:
        print(f"Failed to set emotion: {response.status_code}, {response.text}")


def trigger_small_talk(text, instruction = ''):
    response = requests.post(
        f"{BASE_URL}/trigger_small_talk",
        json={'prompt': text, 'instruction': instruction },  # Send JSON data
        headers={'Content-Type': 'application/json'}  # Ensure proper content type
    )
    return response.json()  # Correctly return parsed JSON response


def get_enable_small_talk():
    response = requests.get(f"{BASE_URL}/get_enable_small_talk")
    return response.json()



def trigger_speech(text):
    response = requests.post(
        f"{BASE_URL}/trigger_speech",
        json={'speech': text},  # Send JSON data
        headers={'Content-Type': 'application/json'}  # Ensure proper content type
    )
    return response.json()  # Correctly return parsed JSON response


def get_speech():
    response = requests.get(f"{BASE_URL}/get_speech")
    return response.json()


def encode_result(result_pos):
    result_obj = []
    for result in result_pos:
        result = [ str(r) for r in result ]
        result_obj.append(' '.join(result))
    result_str = ' '.join(result_obj)
    return result_str

def decode_result(result_str):
    results = result_str.split(' ')
    result_list = []
    for i in range(0, len(results), 4):
        result_list.append(results[i: i + 4])
    return result_list


def get_strategy(text):
    response = requests.post(
        f"{BASE_URL}/qa_strategy",
        json={'text': text},  # Send JSON data
        headers={'Content-Type': 'application/json'}  # Ensure proper content type
    )
    try:
        strategy = response.json()['response'].split(',')
        strategy = [float(i.strip()) for i in strategy]
    except:
        return [0.0, 0.0, 0.0, 120.0]
    return strategy


# Example usage
# set_emotion("excited")
# get_emotion()

# res = set_state()
# print(res)

# res = progress()
# print(res)

result_pos = [ ['orange', 14.9, -44.2, 1], ['blue', 78.3, 17.6, 1]]
# result_str = encode_result(result_pos)
# res = set_result(result_str)
# print(res)

# result_str = get_result()['result_pos']
# result_list = decode_result(result_str)
# print(result_list)

test_prompt = '''Generate the 4 parameters only without any description.
Below is the current configuration:
'''
for p in result_pos:
    test_prompt += f'{p[0]}: {p[1]}, {p[2]}\n'
test_prompt += '\nExample: 12, -10, 60, 150\nEffect: It shoots to the left side with huge force.'
print('[Given Prompt]')
print(test_prompt)
res = get_strategy(test_prompt)
print('-' * 42)
print('[Generated Strategy]')
print(res)



# res = trigger_small_talk(test_prompt)
# print(res)
# res = get_enable_small_talk()
# print(res)


# res = trigger_speech('Ohhhh nooo... I am so sad..')
# print(res)

# res = get_speech()
# print(res)