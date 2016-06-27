import traci
from collections import Counter

class RouteController:

    def __init__(self, link_index_to_turning_direction, in_lane_and_out_lane_to_link_index):
        self._link_index_to_turning_direciton = link_index_to_turning_direction
        self._in_lane_and_out_lane_to_link_index = in_lane_and_out_lane_to_link_index

    def get_destination(self, veh_id):
        route = traci.vehicle.getRoute(veh_id)
        return route.pop()

    def get_next_lane_pair(self, veh_id):
        route = traci.vehicle.getRoute(veh_id)
        if len(route) == 1:
            return 0
        else:
            in_lane, out_lane = route[0:2]
            return in_lane, out_lane

    def get_veh_link_index(self, veh_id):
        in_lane, out_lane = self.get_next_lane_pair(veh_id)
        return self._in_lane_and_out_lane_to_link_index[in_lane][out_lane]

    def get_veh_turning_direction(self, veh_id):
        in_lane, out_lane = self.get_next_lane_pair(veh_id)
        return self._link_index_to_turning_direction[self._in_lane_and_out_lane_to_link_index[in_lane][out_lane]]

    def get_queue_length_per_link_index(self, lanes):
        veh_ids  = [traci.getLastStepVehicleIDs(lane_id) for lane_id in lanes]
        veh_link_indexes = [self.get_veh_link_index(veh) for veh in veh_ids]
        return Counter(veh_link_indexes)

class RouteControllerContainer:
    def __init__(self):
        self._container = []

    def get_queue_length_per_link_index(self, tls_id, lanes):
        pass