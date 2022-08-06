classdef ARPOD_Statistics
    properties
        initTraj = [0;0;0;0;0;0];
        fuelConsumed = 0;
        currTraj = [0;0;0;0;0;0];
        
        trackTraj = [];
        trackU = [];
        trackFuelConsumption = [];
        timestamps = [];
    end
    methods
        function none = initBenchmark(obj)
            return;
        end
        function none = updateBenchmark(obj)
            return;
        end
        function none = graph(obj)
            return;
        end
    end
end