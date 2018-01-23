function f = ownOutFlow (self, Sout, time)
    
    % Outflow computation
    if self.computedOutFlow ~= -1
        f = self.computedOutFlow;
    else
        outRoads = self.myNet.neighborsOut(self.id);
        minimum = 1e10;
        if length(outRoads)>=1
            supply  = self.myNet.roads(outRoads(1)).getSupply;
            beta    = self.myNet.turnings(self.id, outRoads(1));
            minimum = min(minimum,supply/beta);
            for k = 2 : length(outRoads)
                minimum = min (minimum,...
                    self.myNet.roads(outRoads(k)).getSupply/self.myNet.turnings(self.id, outRoads(k)));
            end
        end

        if minimum == 1e10
            f = min( self.getDemand, Sout(self.id, time));
        else
            f = min (minimum(:), self.getDemand);
        end
        self.computedOutFlow = f;
    end
end