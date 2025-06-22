from utils import *


result_pos = [ ['blue', 2, 1, 1], ['blue', 1, 1, 1], ['blue', 2, 2, 2]]
# result_str = encode_result(result_pos)
# res = set_result(result_str)
# print(res)

# result_str = get_result()['result_pos']
# result_list = decode_result(result_str)
# print(result_list)

test_prompt = '''
Below is the current configuration:
'''
for p in result_pos:
    test_prompt += f'{p[0]}: {p[1]}, {p[2]}\n'
res = get_strategy(test_prompt)
print(res)