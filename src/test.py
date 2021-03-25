from time import perf_counter_ns


def map_to_hex(bin_to_hex_converter_dict, p_str):
    p_hex = ''
    temp_pstr = ''
    for index in range(0, len(p_str)):
        temp_pstr += p_str[index]
        if index % 4 == 3:
            p_hex += bin_to_hex_converter_dict[temp_pstr]
            temp_pstr = ''
    return p_hex


binToHexConverterDict = {
    '0000': '0',
    '0001': '1',
    '0010': '2',
    '0011': '3',
    '0100': '4',
    '0101': '5',
    '0110': '6',
    '0111': '7',
    '1000': '8',
    '1001': '9',
    '1010': 'A',
    '1011': 'B',
    '1100': 'C',
    '1101': 'D',
    '1110': 'E',
    '1111': 'F'
}

explored_map = '11111000000000000111000000000000111111111100000111111100000000111000000000000111000000000000000000000' \
               '00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000' \
               '000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000011'

obstacle_map = '00000000000000000000001000000000'

t1_start = perf_counter_ns()

hex_explored = map_to_hex(binToHexConverterDict, explored_map)

t1_stop = perf_counter_ns()

print(hex_explored)
print(f'Elapsed time in ns: {t1_stop - t1_start}\n')

t2_start = perf_counter_ns()

hex_explored_2 = hex(int(explored_map, 2)).lstrip('0x').upper()

t2_stop = perf_counter_ns()

print(hex_explored_2)
print(f'Elapsed time in ns: {t2_stop - t2_start}\n')

t3_start = perf_counter_ns()
hex_obstacle = map_to_hex(binToHexConverterDict, obstacle_map)
t3_stop = perf_counter_ns()

print(hex_obstacle)
print(f'Elapsed time in ns: {t3_stop - t3_start}\n')

t4_start = perf_counter_ns()
hex_obstacle_2 = '{0:0{1}X}'.format(int(obstacle_map, 2), int(len(obstacle_map) / 4))
t4_stop = perf_counter_ns()

print(hex_obstacle_2)
print(f'Elapsed time in ns: {t4_stop - t4_start}\n')

unpadded_obstacle_map = '00000000000000000000001000000'
extra = len(unpadded_obstacle_map) % 8
padding = 0
if extra != 0:
    padding = 8 - extra

t5_start = perf_counter_ns()
for i in range(0, padding):
    unpadded_obstacle_map += '0'
t5_stop = perf_counter_ns()

print(unpadded_obstacle_map)
print(f'Elapsed time in ns: {t5_stop - t5_start}\n')

unpadded_obstacle_map = '00000000000000000000001000000'
t6_start = perf_counter_ns()
unpadded_obstacle_map = unpadded_obstacle_map.ljust(padding + len(unpadded_obstacle_map), '0')
t6_stop = perf_counter_ns()

print(unpadded_obstacle_map)
print(f'Elapsed time in ns: {t6_stop - t6_start}\n')

unpadded_obstacle_map = '00000000000000000000001000000'
t7_start = perf_counter_ns()
for i in range(0, padding):
    unpadded_obstacle_map += '0'
p1 = hex(int(explored_map, 2)).lstrip('0x').upper()
p2 = '{0:0{1}X}'.format(int(unpadded_obstacle_map, 2), int(len(unpadded_obstacle_map) / 4))
t7_stop = perf_counter_ns()

print(p1, p2)
print(f'Elapsed time in ns: {t7_stop - t7_start}\n')

unpadded_obstacle_map = '00000000000000000000001000000'
t8_start = perf_counter_ns()
for i in range(0, padding):
    unpadded_obstacle_map += '0'
p1 = map_to_hex(binToHexConverterDict, explored_map)
p2 = map_to_hex(binToHexConverterDict, unpadded_obstacle_map)
t8_stop = perf_counter_ns()

print(p1, p2)
print(f'Elapsed time in ns: {t8_stop - t8_start}\n')
