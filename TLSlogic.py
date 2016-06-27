from sumolib import net
from collections import defaultdict

def print_output(func):
    def wrapper(*args, **kwargs):
        output = func(*args, **kwargs)
        print(func, args, kwargs, output)
        return output
    return wrapper

def get_compatible_lanes_matrix_and_phases_from_net_file(net_file):
    """reads the given net file and returns a dict with TLS ids as the keys and matrices containing the L matrix and
    the phase settings"""
    netObj = net.readNet(net_file, withPrograms=True)

    TLS_phases = {}
    TLS_L = {}

    for TLS in netObj._tlss:

        TLS_ID = TLS.getID()

        phases = []
        for program in TLS._programs:
            for phase_settings, _ in TLS._programs[program]._phases:
                if 'G' in phase_settings and phase_settings not in phases:
                    phases.append(phase_settings)

        L = []
        for index, phase in enumerate(phases):
            L.append([])
            for letter in list(phase):
                if letter == 'r':
                    L[index].append(0)
                else:
                    L[index].append(1)

        TLS_phases.update({TLS_ID: phases})
        TLS_L.update({TLS_ID: L})

    return TLS_L, TLS_phases

def get_in_out_lanes_to_index(net_file):
    netObj = net.readNet(net_file, withPrograms=True)

    TLS_in_lanes = {}
    TLS_out_lanes = {}

    for TLS in netObj._tlss:

        TLS_ID = TLS.getID()

        in_entry = ['__' for conn in TLS._connections]
        out_entry = ['__' for conn in TLS._connections]

        for conn in TLS._connections:
            in_entry[conn[2]] = conn[0].getID()
            out_entry[conn[2]] = conn[1].getID()

        TLS_in_lanes.update({TLS_ID: in_entry})
        TLS_out_lanes.update({TLS_ID: out_entry})

    return TLS_in_lanes, TLS_out_lanes

def get_connection_to_turn_defs(net_file):
    netObj = net.readNet(net_file, withConnections=True)

    TLS_directions_by_link_index =  defaultdict(dict)
    TLS_link_index_by_in_lane_and_out_edge = defaultdict(dict)

    for TLS in netObj._tlss:

        directions_by_link_index  = []
        link_index_by_in_lane_and_out_edge = defaultdict(defaultdict)

        data_tuples = []

        for in_lane, out_lane, link_index in TLS._connections:

            for connection in in_lane._outgoing:
                link = connection._tlLink

                if link == link_index : data_tuples.append((in_lane, out_lane, link_index, connection._direction))

        data_tuples = sorted(data_tuples, key=lambda entry: entry[2])

        for entry in data_tuples:
            directions_by_link_index.append(entry[3])
            link_index_by_in_lane_and_out_edge[entry[0].getID()][entry[1]._edge.getID()] = entry[2]

        TLS_directions_by_link_index[TLS.getID()] = directions_by_link_index
        TLS_link_index_by_in_lane_and_out_edge[TLS.getID()] = link_index_by_in_lane_and_out_edge

    return TLS_directions_by_link_index, TLS_link_index_by_in_lane_and_out_edge




get_connection_to_turn_defs('2LaneGrid.net.xml')



# for ii in range(2**num - 1):
#
#     max_index = 0
#
#     for index, jj in enumerate(output):
#
#         if jj == 0 and max_index < index : max_index = index
#
#     output[max_index] = 1
#
#     for index in range(max_index+1, len(output)):
#
#         output[index] = 0
#
#     possibilities.append(output[:])

# for node in netObj.getNodes():
#     L = []
#     R = []
#     F = []
#     for index in node._foes:
#         foes_as_string = node._foes[index]
#         response_as_string = node._prohibits[index]
#
#         foes_as_int = list(reversed([int(char) for char in list(foes_as_string)]))
#         response_as_int = list(list(reversed([int(char) for char in list(response_as_string)])))
#
#         L_row = [int(not(entry)) for entry in foes_as_int]
#         L.append(L_row)
#         F.append(foes_as_int)
#         R.append(response_as_int)
#
#     true_possibilities = []
#
#     if not(L) : continue
#     if not(R) : continue
#
#     #print(R)
#     combination_strings = []
#     for combination_to_test in possibilities:
#
#         combo_string = ['r' for _ in combination_to_test]
#
#         status = True
#
#         for index, queue_unlocked in enumerate(combination_to_test):
#             index_dir = directions[index]
#
#             # If queue is unlocked, check it against every other queue unlocked
#             if queue_unlocked:
#
#                 for other_index, other_queue_unlocked in enumerate(combination_to_test):
#                     other_index_dir = directions[other_index]
#
#                     if other_queue_unlocked:
#
#                         # ii has foe jj, and jj has priority at unsignalled junctions
#                         if F[index][other_index] and (R[index][other_index] or R[other_index][index]):
#                             # if ii is turning right, it has priority over left turns. It will not conflict with other right turns, and conflicting straight flows will be red.
#                             if index_dir == 'r' and other_index_dir == 'l':
#                                 if combo_string[index] != 'g' : combo_string[index] = 'G'
#                             # if ii is going straight, it has priority. If jj is going straight, it must have a red light, if jj is turning left it may have a green light but give way.
#                             # If jj is turning right it will have a red light, or it will give way.
#                             elif index_dir == 's' and (other_index_dir == 's' or other_index_dir == 'r'):
#                                 status = False
#                             elif index_dir == 's' and other_index_dir == 'l':
#                                 status = False
#                             # if ii is turning left it will give way to straight flows and right turning flows. If jj is left turning flow it will be red.
#                             elif index_dir == 'l' and (other_index_dir == 's' or other_index_dir == 'r'):
#                                 combo_string[index] = 'g'
#                             elif index_dir == 'l' and other_index_dir == 'l':
#                                 status = False
#                         elif not(F[index][other_index]):
#                             if combo_string[index] != 'g': combo_string[index] = 'G'
#                         else:
#                             status = False
#
#         if status:
#             true_possibilities.append(combination_to_test)
#             combination_strings.append(combo_string)
#
#     np.set_printoptions(threshold='nan')
#     print(np.array(true_possibilities))
#     print(np.array(combination_strings))
#
#
#     sums = [sum(possibility) for possibility in true_possibilities]
#     index_with_sum = [(index, value) for index, value in enumerate(sums)]
#
#     a = sorted(index_with_sum, key=lambda (index, value) : value, reverse=True)
#
#     print(str(combination_strings[a[0][0]]))

    # num_lanes = range(8)
    # lanes_to_index = np.array([[1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    #       [0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])


# for entry_number, entry in enumerate(row):
#
#     if combination_to_test[row_number] and entry





#     for row_number, row in enumerate(L):
#         branches = []
#         phase = []
#         compatible_flows = [[0 for _ in range(len(row))] for _ in range(len(row))]
#
#         for entry_number, entry in enumerate(row):
#
#             if entry:
#
#                 compatible_flows[entry_number][row_number] = 1
#                 compatible_flows[entry_number][entry_number] = 1
#
#                 for another_entry_number, another_entry in enumerate(row):
#
#                     if row[row_number] and L[entry_number][another_entry_number]:
#
#                         compatible_flows[entry_number][another_entry_number] = 1
#
#         print(np.array(compatible_flows))
#
#         combined_flows = [[1 for _ in range(len(row))] for _ in range(len(row))]
#
#         for cf_row_number, cf_row in enumerate(compatible_flows):
#
#             for cf_entry_number, cf_entry in enumerate(row):
#
#                 if cf_entry:
#
#                     for cf_another_entry_number, cf_another_entry in enumerate(row):


