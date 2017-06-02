"""

Date of creation: 2017/02/21
Date of completion (1st time):

Description: This is the main module for taxi ride-sharing simulation.

"""



from road_network import load_data
from spatio_temporal_index import SpatioTemporalDatabase
from query import load_query, init_schedule_node
from taxi import gen_taxi
from  msdispatcher import Dispatcher ###########################################################
from constants import SIM_START_TIME, SIM_END_TIME, WAITING, CANCELLED,DEBUG_MODEL
from container import PriorityQueue
from datetime import datetime
from location import get_distance
import time

import winsound


class Simulation:
    """
    This is a class which is responsible for setting up and running a simulation.
    """
    def __init__(self):
        self.maxLoad = list()
        self.sumLoad = list()
        road_network = load_data()
        self.road_network = road_network

        db = SpatioTemporalDatabase()
        db.load_road_network(road_network)  #To grid city
        db.init_static_info(road_network)   #To compute Matrix
        self.taxi_set = gen_taxi(db, self.road_network)  #To allocate taxi to grid based on vertex
        db.init_dynamic_info(self.taxi_set, SIM_START_TIME)#Initialize the taxi list of grid cell
        self.db = db #

        [self.query_set, self.query_queue] = load_query()
        init_schedule_node(self.query_queue, self.road_network, self.db)

        self.dispatcher = Dispatcher()

    def run(self):
        start_time = time.clock()
        query_num =  len(self.query_set)
        print "query to be processed NUM:",query_num
        processed_order = 0
        query_sum_dist = 0

        waiting_queries = PriorityQueue()

        for timestamp in range(SIM_START_TIME, SIM_END_TIME+1):
            print("Time: %d" % timestamp)
            sim_start_time = time.clock()
            # Catch the queries to be processed in this timestamp. The queries consists of two parts:
            # 1. queries that happened in this timestamp
            # 2. queries that stranded in previous timestamps
            while not self.query_queue.empty():
                new_query = self.query_queue.get()
                if new_query.timestamp == timestamp:
                    waiting_queries.put(new_query, new_query.timestamp)
                else:
                    self.query_queue.put(new_query, new_query.timestamp)
                    break
            while not self.dispatcher.failed_queries.empty():
                old_query = self.dispatcher.failed_queries.get()
                waiting_queries.put(old_query, old_query.timestamp)

            # Process the queries.
            while not waiting_queries.empty():
                query = waiting_queries.get()
                if query.status == CANCELLED:
                    self.dispatcher.add_cancelled_query(query)
                else:
                    if DEBUG_MODEL:
                        print "......To process query:", query.id
                        print "origin: %s ---> dest:%s" %(query.o_geohash, query.d_geohash)
                        print "[%f, %f]----->[%f,%f]" %(query.pickup_window.early,query.pickup_window.late, query.delivery_window.early, query.delivery_window.late)
                        print "[%f, %f]----->[%f,%f]"%(query.origin.lon, query.origin.lat, query.destination.lon, query.destination.lat)
                        print "dist:",get_distance(query.origin, query.destination)

                    flag_suc = self.dispatcher.dispatch_taxi(timestamp, query, self.db, self.taxi_set, self.road_network, self.query_set)
                    if  flag_suc:
                        processed_order += 1
                        query_sum_dist += get_distance(query.origin, query.destination)
                        print "order Num:", processed_order
                        print "order Dist:", query_sum_dist


                    if processed_order > 0 and processed_order % 100 == 0:
                        self.print_load(processed_order)

            # Update the status of all the queries
            for query in self.query_set.values():
                if query.timestamp <= timestamp and query.status == WAITING:
                    query.update_status(timestamp)

            # All the taxis drive according to their schedule.
            for taxi in self.taxi_set.values():
                taxi.drive(timestamp, self.road_network, self.dispatcher, self.query_set, self.db)

            print "Done in %f seconds" % (time.clock()-sim_start_time)

        [max,min,count] = self.print_utility()
        print("after:"), max - min,("  count:"),count
        print ("satisfy ratio:"), self.print_satisfy(query_num)
        for load in self.maxLoad:
            print load,"    "
        for load in self.sumLoad:
            print load,"    "
        print("The simulation is end. Elapsed time is %f." % (time.clock() - start_time))
        winsound.Beep(600, 1000)


    def print_utility(self):
        '''
        :return: [max_driving_distance, min_driving_distance, count]
        '''
        min = float('inf')
        max = float('-inf')
        count = 0
        for taxi_id in self.taxi_set:
            taxi = self.taxi_set[taxi_id]
            if taxi.driving_distance > max:
                max = taxi.driving_distance

            if taxi.driving_distance < min:
                min = taxi.driving_distance

            if taxi.driving_distance > 0:
                count += 1

        return [max, min, count]

    def print_load(self, num_order):
        if len(self.maxLoad) < num_order/100:
            max = 0;
            sum = 0;
            for taxi_id in self.taxi_set:
                taxi = self.taxi_set[taxi_id]
                if taxi.load > max:
                    max = taxi.load
                sum += taxi.load;
            print "====> ordernum[%d]:%f <--->  %f  " %(num_order, max, sum)
            self.maxLoad.append(max)
            self.sumLoad.append(sum)
        else:
            pass


    def print_satisfy(self, total):
        count = 0
        for taxi_id in self.taxi_set:
            taxi = self.taxi_set[taxi_id]
            count += taxi.num_order
        print "query_num:", total,"   satisfied count:",count
        return count*1.0/total

if __name__ == "__main__":
    print datetime.now()
    print 'Start init...', datetime.now()
    sim = Simulation()
    print datetime.now(), '-------finish init---------------'
    winsound.Beep(600, 1000)

    print("The simulation system is running...")

    sim.run()
    print datetime.now()