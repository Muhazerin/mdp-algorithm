def fpParser(system, path_data):
    # Example of path_data --> (1,1,N);(2,1,N);(2,2,E);(2,12,E);(3,12,N);(18,13,N)
    # (y,x,dir)
    hori = ['E', 'W']
    vert = ['N', 'S']
    step_seq = []

    dir_move_map = {'N': 'f', 'E': 'tr', 'S': 'r', 'W': 'tl'}

    path_seq = path_data.split(';')
    curr_path = path_seq[0]

    # get rid of brackets (1,1,N) --> 1,1,N
    init_x, init_y, init_dir = curr_path[slice(1, len(curr_path) - 1)].split(',')
    # supposed to be init_y, init_x, init_dir

    for index, path in enumerate(path_seq):
        y_1, x_1, dir_1 = path[slice(1, len(path) - 1)].split(',')

        if (index + 1) == len(path_seq):
            break
        else:
            next_path = path_seq[index + 1]
            y_2, x_2, dir_2 = next_path[slice(1, len(next_path) - 1)].split(',')
            if system == 'android':
                if dir_2 in vert:
                    step_count = abs(int(y_1) - int(y_2))
                elif dir_2 in hori:
                    step_count = abs(int(x_1) - int(x_2))
                step_seq = step_seq + [dir_move_map[dir_2] for x in range(step_count)]

            elif system == 'arduino':
                if dir_1 == dir_2:
                    if dir_1 in vert:
                        step_count = abs(int(y_1) - int(y_2))
                    elif dir_1 in hori:
                        step_count = abs(int(x_1) - int(x_2))
                    step_seq = step_seq + ['w' for x in range(step_count)]
                else:
                    if (dir_1 == 'N' and dir_2 == 'E') \
                            or (dir_1 == 'S' and dir_2 == 'W') \
                            or (dir_1 == 'E' and dir_2 == 'S') \
                            or (dir_1 == 'W' and dir_2 == 'N'):
                        step_seq.append('d')
                    elif (dir_1 == 'N' and dir_2 == 'W') \
                            or (dir_1 == 'S' and dir_2 == 'E') \
                            or (dir_1 == 'E' and dir_2 == 'N') \
                            or (dir_1 == 'W' and dir_2 == 'S'):
                        step_seq.append('a')

                    if dir_2 in vert:
                        step_count = abs(int(y_1) - int(y_2))
                    elif dir_2 in hori:
                        step_count = abs(int(x_1) - int(x_2))
                    else:
                        print('Error: ' + str(dir_1) + ', ' + str(dir_2))
                    step_seq = step_seq + ['w' for x in range(step_count)]

    if system == 'android':
        return 'FP|{x}|{y}|{d}|{steps}'.format(x=init_x, y=init_y, d=init_dir, steps=','.join(step_seq))
    elif system == 'arduino':
        step_seq = compressStepSeq(step_seq)
        return '{steps}'.format(steps=''.join(step_seq))


def compressStepSeq(step_seq):
    forward_count = 0
    seq = []
    for step in step_seq:
        if step == 'w':
            forward_count = forward_count + 1
            if forward_count == 9:
                seq.append('w{x}'.format(x=forward_count))
                forward_count = 0
        else:
            seq.append('w{x}'.format(x=forward_count))
            seq.append(step)
            forward_count = 0
    if forward_count > 0:
        seq.append('w{x}'.format(x=forward_count))
    return seq


def mdfParser(system, mdf_string):
    mdf_data = mdf_string.split('|')
    if system == 'android':
        return '|'.join(mdf_data[0: len(mdf_data) - 1])
    if system == 'arduino':
        return mdf_data[len(mdf_data) - 1]


def pcMsgParser(msg):
    data = msg.split('|')
    command = data[0]

    target = None
    payload = msg

    if command == 'FP':
        target = 'both'
        payload = {
            'android': fpParser('android', data[1]),
            'arduino': fpParser('arduino', data[1]),
        }

    elif command == 'MDF':
        target = 'both'
        payload = {
            'android': mdfParser('android', msg),
            'arduino': mdfParser('arduino', msg),
        }

    elif command == 'EC':
        target = 'both'
        payload = {
            'android': payload,
            'arduino': payload,
        }

    elif command == 'TP':
        target = 'rpi'
        payload = command

    # elif command in arduino_commands:
    #     target = 'arduino'
    #     payload = command

    else:
        print('pcMsgParser unknown command: ' + str(command))
        return None

    return {
        'target': target,
        'payload': payload
    }


if __name__ == '__main__':
    system = 'android'
    path_data = '(1,1,N);(2,1,N);(2,1,E);(2,2,E);(2,3,E)'
    print(f'path_data: {path_data}')
    print(f'android: {fpParser(system, path_data)}')
    system = 'arduino'
    print(f'arduino: {fpParser(system, path_data)}')
    print('\nMDF')
    msg = "MDF|Test"
    print(f'msg: {msg}')
    print(f'{pcMsgParser(msg)}')
