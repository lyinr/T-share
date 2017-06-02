

from road_network import get_shortest_path
from location import get_distance
from constants import AVERAGE_SPEED, DEBUG_MODEL
from road_network import dijkstra
from container import Queue
from road_network import RoadNetwork
from spatio_temporal_index import SpatioTemporalDatabase
from query import Query
import winsound
import time


class Dispatcher:

    def __init__(self,
                 failed_queries=None,
                 waiting_queries=None,
                 completed_queries=None,
                 cancalled_queries=None):
        '''
        Initialize a Dispatcher.

        :param failed_queries: a Queue stores the queries that the dispatcher failed to find a taxi, these queries
        need to be processed in the next timestamp
        :param waiting_queries: a dict stores the queries that are dispatched a taxi and under waiting
        :param completed_queries: a list stores successfully completed queries
        :param cancelled_queries: a list stores cancelled queries
        :type failed_queries: Queue[Query]
        :type waiting_queries: dict[int, Query]
        :type completed_queries: list[Query]
        :type cancelled_queries: list[Query]
        :return: None
        '''

        if failed_queries is None:
            self.failed_queries = Queue()
        else:
            self.failed_queries = failed_queries

        if waiting_queries is None:
            self.waiting_queries = dict()
        else:
            self.waiting_queries = waiting_queries

        if completed_queries is None:
            self.completed_queries = list()
        else:
            self.completed_queries = completed_queries

        if cancalled_queries is None:
            self.cancelled_queries = list()
        else:
            self.cancelled_queries = cancalled_queries

    def add_cancelled_query(self, query):
        '''
        cancel a query.
        :param query: a Query instance
        :type query: Query
        :return: None
        '''
        self.cancelled_queries.append(query)

    def add_waiting_query(self, query):
        '''
        Cancel a query.
        :param query: a Query instance
        :type query: Query
        :return: None
        '''
        self.waiting_queries[query.id] = query

    def add_serving_query(self, query):
        '''
        :param query: a Query instance
        :type query: Query
        :return: None
        '''
        self.waiting_queries.pop(query.id)

    def add_failed_query(self, query):
        '''
        :param query: a Query instance
        :type query: Query
        :return: None
        '''
        self.failed_queries.put(query)

    def add_completed_query(self, query):
        '''
        :param query:  a Query instance
        :type query: Query
        :return: None
        '''
        self.completed_queries.append(query)

    def dispatch_taxi(self, timestamp, query, database, taxi_set, road_network, query_set):
        '''
        Respond to the query and try to dispatch a taxi for the query.

        :param timestamp: the current time of the simulation system
        :param query: the query
        :param database: the spatio-temporal database
        :param taxi_set: the taxi set
        :param road_network: the road network
        :param query_set:
        :type timestamp: int
        :type query: Query
        :type database: SpatioTemporalDatabase
        :type taxi_set: dict[int, Taxi]
        :type road_network: RoadNetwork

        :return: if the dispatch is successful or not
        :rtype: void
        '''
        #print("searching taxi_list")
        candi_taxi_list = self.dual_side_taxi_searching(timestamp,query,database, road_network,taxi_set)

        # if DEBUG_MODEL:
        #     if candi_taxi_list:
        #         print("candi_taxi_list successfully.......................taxi coming")
        #     else:
        #         print('candi_taxi_list failed..')

        if candi_taxi_list and self.__schedule(timestamp,query,candi_taxi_list,taxi_set,database,road_network,query_set):
            self.add_waiting_query(query)
            if DEBUG_MODEL:
                print( ".............................satisfy the query ......")

            return True

        else:
            if DEBUG_MODEL and not candi_taxi_list:
                print "Failed: No taxi 000000000000000"
            self.add_failed_query(query)
            return False

        if DEBUG_MODEL:
            winsound.Beep(600, 1000)


    def dual_side_taxi_searching(self, timestamp, query, database, road_network, taxi_set):
        '''
        Dual_side taxi searching.
        :param timestamp: current time of the simulation
        :param query: the query
        :param database: the query
        :type timestamp: int
        :type query: Query
        :type database: sptioTemporalDatabase
        :return: a list of candidate taxis
        :type: list[int]
        '''
        o_grid = query.o_geohash
        o_candi_taxi_list = []
        d_grid = query.d_geohash
        d_candi_taxi_list = []
        candi_taxi_list = []

        #print("searching grid........")
        o_time_grid_list = self.grid_search(timestamp, query, True, database, road_network)
        d_time_grid_list = self.grid_search(timestamp, query, False, database, road_network)

        o_grid_list = [grid[0] for grid in database.grid[o_grid].spatial_grid_list if grid[0] in o_time_grid_list]
        d_grid_list = [grid[0] for grid in database.grid[d_grid].spatial_grid_list if grid[0] in d_time_grid_list]
        # print o_grid_list

        if DEBUG_MODEL and o_grid_list:
            print("o_grid_list successfully......."), len(o_grid_list)
            o_taxi_num = 0
            for grid_id in o_grid_list:
                o_taxi_num += len(database.grid[grid_id].taxi_list)
            print "Taxi num:", o_taxi_num

        if DEBUG_MODEL and d_grid_list:
            print("d_grid_list successfully......."), len(d_grid_list)
            d_taxi_num = 0
            for grid_id in d_grid_list:
                d_taxi_num += len(database.grid[grid_id].taxi_list)
            print "Taxi num:",d_taxi_num

        o_stop = False
        d_stop = False
        o_index = 0
        d_index = 0
        o_len = len(o_grid_list)
        d_len = len(d_grid_list)
        while not candi_taxi_list and (o_stop == False or d_stop == False):
            if o_index < o_len:
                o_tem_grid_id = o_grid_list[o_index]
                o_index += 1
                for o_tem_taxi in database.grid[o_tem_grid_id].taxi_list:  #:type: taxi_list dict()
                    o_tem_time = database.grid[o_tem_grid_id].taxi_list[o_tem_taxi]
                    #print len(database.grid[o_tem_grid_id].taxi_list), "....." , timestamp, "...", o_tem_time, "...",query.pickup_window.late
                    if o_tem_time < query.pickup_window.late :
                        o_candi_taxi_list.append(o_tem_taxi)
                    else:
                        break
            else:
                o_stop = True

            if d_index < d_len:
                d_tem_grid_id = d_grid_list[d_index]
                d_index += 1
                for d_tem_taxi in database.grid[d_tem_grid_id].taxi_list:
                    d_tem_time = database.grid[d_tem_grid_id].taxi_list[d_tem_taxi]
                    if d_tem_time < query.delivery_window.late:
                        d_candi_taxi_list.append(d_tem_taxi)
                    else:
                        break
            else:
                d_stop = True

            if DEBUG_MODEL:
                print("......O_candi_taxi:"), len(o_candi_taxi_list)
                print("......d_candi-taxi:"),len(d_candi_taxi_list)
            candi_taxi_list = [val for val in o_candi_taxi_list if val in d_candi_taxi_list]

        #print("..............index"), o_index,"/",len(o_grid_list), "......", d_index,"/",len(d_grid_list)
        if not candi_taxi_list:
            for taxi_id in o_candi_taxi_list:
                if not taxi_set[taxi_id].schedule:
                    candi_taxi_list.append(taxi_id)
        return candi_taxi_list

    def __schedule(self, timestamp, query, candi_taxi_list, taxi_set, database, road_network, query_set):
        '''
        Taxi scheduling.

        The purpose of scheduling is to insert the origin and destination (ScheduleNode) of the query into the schedule
        of the taxi which satisfies the query with minimum additional travel distance.

        :param timestamp:
        :param query:
        :param candi_taxi_list: list[int]
        :param taxi_set: dict[int,taxi]
        :param database
        :param road_network:
        :param query_set:
        :return:
        '''
        if len(candi_taxi_list) == 0:
            return False

        min_taxi_o_index = -1
        min_taxi_d_index = -1
        min_taxi_increase = float('inf')
        min_taxi_load = float('inf')
        min_taxi_distance = 0
        min_matched_taxi_id = -1

        for taxi_id in candi_taxi_list:
            taxi = taxi_set[taxi_id]
            if not taxi.is_available():
                continue

            o_index = -1
            d_index = -1
            min_increase = float('inf')
            min_load = float('inf')
            cur_schedule_distance = taxi.schedule_distance
            matched_taxi_id = -1
            schedule_len = len(taxi.schedule)

            #computer the distance between schedule nodes
            dist_between_schedule = []
            for i in range(0, schedule_len-1):
                dist_between_schedule.append(get_shortest_path(road_network,taxi.schedule[i].matched_vid, taxi.schedule[i+1].matched_vid).distance)

            for i in range(0, schedule_len+1):
                for j in range(i+1,schedule_len + 2):
                    taxi.schedule.insert(i,query.o_schedule_node)
                    taxi.schedule.insert(j,query.d_schedule_node)

                    [new_schedule_dist, cur_load] = self.insertion_feasibility_check(timestamp, query, i, j, database, taxi, road_network, query_set, dist_between_schedule)
                    taxi.schedule.pop(j)
                    taxi.schedule.pop(i)

                    if 0 == cur_load:
                        continue
                    cur_increase = new_schedule_dist - cur_schedule_distance

                    if  cur_increase < min_increase:
                        o_index = i
                        d_index = j
                        matched_taxi = taxi.id
                        min_increase = cur_increase
                        min_distance = new_schedule_dist
                        min_load = cur_load

            if min_increase < min_taxi_increase:
                min_taxi_o_index = o_index
                min_taxi_d_index = d_index
                min_matched_taxi_id = matched_taxi
                min_taxi_distance = min_distance
                min_taxi_load = min_load

        if min_matched_taxi_id == -1:
            # print "All taxis are...................44444444"
            return False
        else:
            query.matched_taxi = min_matched_taxi_id
            pick_taxi = taxi_set[query.matched_taxi]
            pick_taxi.schedule.insert(min_taxi_o_index, query.o_schedule_node)
            pick_taxi.schedule.insert(min_taxi_d_index, query.d_schedule_node)
            pick_taxi.update_route(road_network)
            pick_taxi.update_schedule_distance(min_taxi_distance, min_taxi_load)
            # print "o_index:%d   d_index:%d"%(min_taxi_o_index, min_taxi_d_index)
            # print "min_load:",min_taxi_load
            # print "taxi[%d]: %f"%(query.matched_taxi, pick_taxi.load)
            # print "schedule:", pick_taxi.schedule[0]
            # print "to-----:", pick_taxi.schedule[1]
            return True

    def grid_search(self, timestamp, query, isOrigin, database, road_network):
        '''
        To search candidate grid for a query
        :param timestamp: current time of system
        :param deadline: latest time of query ( pick/deliver)
        :param database: grid network of city
        :return:list[geohash]
        '''
        grid_list = []
        #compute pickup
        cur_geohash = query.o_geohash
        deadline = query.pickup_window.late
        if ( isOrigin == False):
            cur_geohash = query.d_geohash
            deadline = query.delivery_window.late

        if DEBUG_MODEL:
            print "cur_geohash:",cur_geohash
        for grid in database.grid[cur_geohash].temporal_grid_list:
            # print "timestam:",timestamp,"   grid_2_grid:",grid[1],"     deadline:",deadline
            if timestamp + grid[1] <= deadline:
                grid_list.append(grid[0])
            else:
                break
        return grid_list


    def insertion_feasibility_check(self, timestamp, query,o_index, d_index, database, candi_taxi, road_network, query_set, dist_between_schedule):
        '''
        Return the distance and check the time constrain
        :param timestamp:
        :param query:
        :param o_index:
        :param d_index:
        :param database:
        :param candi_taxi:
        :param road_network:
        :param query_set:
        :return: distance of taxi.schedule
        '''
        # print len(candi_taxi.schedule)
        time_taxi_o = self.time_taxi_to_location(candi_taxi, query.o_schedule_node.matched_vid, road_network, database)
        if timestamp + time_taxi_o > query.pickup_window.late or candi_taxi.num_riders >= 4:
            return [0,0]

        cur_vertex = -1
        min_dist = float('inf')

        pre_vertex_id = None;
        total_dist = 0
        cur_load = 0
        flag_load = 0
        arrive_time =timestamp
        index_shedule_node = -1
        cur_riders = candi_taxi.num_riders
        for schedule_node in candi_taxi.schedule:
            # print "computing ......"
            index_shedule_node += 1
            # print "index:",index_shedule_node
            if schedule_node.is_origin:
                cur_riders += 1
                if cur_riders > candi_taxi.capacity:
                    # print ".......capacity constraint........"
                    return [0,0]
            else:
                cur_riders -= 1
            if(index_shedule_node == 0 ):
                total_dist += get_distance(candi_taxi.location, road_network.get_vertex(schedule_node.matched_vid).location )
                arrive_time = timestamp + total_dist / AVERAGE_SPEED

                if index_shedule_node == o_index:
                    flag_load += 1
            elif(index_shedule_node == o_index or index_shedule_node == d_index):
                path = get_shortest_path(road_network, pre_vertex_id, schedule_node.matched_vid)
                arrive_time += path.distance / AVERAGE_SPEED
                total_dist += path.distance

                if flag_load % 2 == 1:
                    cur_load += path.distance
                flag_load += 1

            elif(index_shedule_node == o_index +1 or index_shedule_node == d_index+1):
                path = get_shortest_path(road_network, pre_vertex_id, schedule_node.matched_vid)
                arrive_time += path.distance / AVERAGE_SPEED
                total_dist += path.distance

                if flag_load % 2 == 1:
                    cur_load += path.distance
            elif(index_shedule_node > d_index) :
                # print "taxi[%f]:  order=>%f"%(candi_taxi.id, candi_taxi.num_order)
                # print "len(schedule)---index:", len(candi_taxi.schedule), "  ", index_shedule_node
                # for node in candi_taxi.schedule:
                #     print node
                # print "len_dist_between_schedule:", len(dist_between_schedule)
                arrive_time += dist_between_schedule[index_shedule_node-3] / AVERAGE_SPEED
                total_dist += dist_between_schedule[index_shedule_node-3]
            elif (index_shedule_node > o_index):
                arrive_time += dist_between_schedule[index_shedule_node-2] / AVERAGE_SPEED
                total_dist += dist_between_schedule[index_shedule_node-2]

                cur_load += dist_between_schedule[index_shedule_node-2]
            else:
                arrive_time += dist_between_schedule[index_shedule_node-1] /AVERAGE_SPEED
                total_dist += dist_between_schedule[index_shedule_node-1]

            # print "index:", index_shedule_node, "    dist:",total_dist
            # time.sleep(120)
            pre_vertex_id = schedule_node.matched_vid

            matched_query_id = schedule_node.query_id
            matched_query = query_set[matched_query_id]
            if schedule_node.is_origin:
                if arrive_time > matched_query.pickup_window.late:
                    # print "......pickup_window......"
                    return [0,0]
            else:
                if arrive_time > matched_query.delivery_window.late:
                    # print "[%f, %f]---[%f,%f]"%(matched_query.pickup_window.early, matched_query.pickup_window.late, matched_query.delivery_window.early, matched_query.delivery_window.late)
                    # print "......deliver_window......",arrive_time
                    return [0,0]

        return [total_dist,cur_load]


    def time_taxi_to_location(self,taxi,vertex_id,road_network,database):
        '''
        :param taxi: Taxi
        :param vertex: query.o_schedule_node.matched_vid/ query.d_schedule_nde.matched_vid
        :param road_network:
        :param database:
        :return: time from taxi to query.o_schedule_node/d_schedule_node
        '''
        #computer the distance from neareast vertex to taxi
        ver_id_2taxi = self.find_taxi_nearest_vertex(taxi,database,road_network)
        ver_2taxi = road_network.get_vertex(ver_id_2taxi)
        taxi_vertex_dist = get_distance(ver_2taxi.location, taxi.location)

        # compute dist from vertex_neareast_taxi to vertex_neareast_loc
        dist_v2v = get_shortest_path(road_network,ver_id_2taxi,vertex_id).distance
        time_taxi_vertex = (taxi_vertex_dist + dist_v2v) / AVERAGE_SPEED

        return time_taxi_vertex

    def find_taxi_nearest_vertex(self,taxi,database,road_network):
        '''
        find the neareast vertex to the taxi
        :param taxi:
        :param database:
        :param road_network:
        :return: vertex_id
        '''
        neareast_vertex = -1
        min_dist  = float('inf')
        for ver_id in database.grid[taxi.geohash].vertex_list:
            vertex = road_network.get_vertex(ver_id)
            dist = get_distance(taxi.location,vertex.location)
            if dist <= min_dist:
                neareast_vertex = ver_id
                min_dist = dist

        return neareast_vertex




