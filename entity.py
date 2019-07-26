#!python3
# -*- coding:utf-8 -*-
import matplotlib.pyplot as plt
import math
import heapq
import time
try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

class Entity:
    def __init__(self, type, name, x=0.0, y=0.0, r=0, x1=0.0, x2=0.0, y1=0.0, y2=0.0):
        self.type = type 
        self.name = name
        if type == 'Point':
            self.x = x 
            self.y = y
        elif type == 'Obstacle':
            self.x = x
            self.y = y
            self.r = r
        elif type == 'Gate':
            self.x1 = x1
            self.y1 = y1
            self.x2 = x2
            self.y2 = y2

class Record:
    def __init__(self, loc=-1, dis=0, gate=0, path=''):
        self.loc = loc
        self.dis = dis
        self.gate = gate
        self.path = path
        self.hash = str(loc) + '$' + str(self.gate)

    def __lt__(self, other):
        return self.dis < other.dis

class Heap:
    def __init__(self):
        self.heap = []
        self.hash = {}

    def push(self, record):
        dis = self.hash.get(record.hash, 1e+10)
        if dis <= record.dis + 1e-6:
            return 
        else:
            self.hash[record.hash] = record.dis
            heapq.heappush(self.heap, (record.dis, record))
    
    def top(self):
        dis, record = self.heap[0]
        return record 

    def pop(self):
        dis, record = heapq.heappop(self.heap)
        return record

class Search:
    def __init__(self, entities, clientID):
        self.entities = entities 
        self.clientID = clientID

    def distance_between_points(self, p1, p2):
        return (p1.x - p2.x)**2 + (p1.y - p2.y)**2

    def check_point_in_obstacle(self, point, obstacle):
        return self.distance_between_points(point, obstacle) <= obstacle.r**2 

    def check_insection_with_obstacle(self, point1, point2, obstacle):
        p1_to_o = math.sqrt(self.distance_between_points(point1, obstacle))
        p2_to_o = math.sqrt(self.distance_between_points(point2, obstacle))
        p1_to_p2 = math.sqrt(self.distance_between_points(point1, point2))
        half = (p1_to_o + p2_to_o + p1_to_p2) / 2.0
        s = math.sqrt(half * (half - p1_to_o) * (half - p2_to_o) * (half - p1_to_p2))
        high = s * 2 / p1_to_p2 
        p1_b = math.sqrt(p1_to_o**2 - high**2)
        p2_b = math.sqrt(p2_to_o**2 - high**2)
        if abs(p1_b + p2_b - p1_to_p2) < 1e-4:
            dis = high 
        else:
            dis = min(p1_to_o, p2_to_o)
        return dis < obstacle.r
    
    def draw(self, type='A'):
        if type == 'A':
            for entity in self.entities:
                if entity.type == 'Point':
                    if entity.name == 'Target0':
                        plt.scatter(entity.x, entity.y, c='r')
                    elif entity.name == 'Target1':
                        plt.scatter(entity.x, entity.y, c='g')
                    elif entity.name == 'Target2':
                        plt.scatter(entity.x, entity.y, c='b')
                    else:
                        print (entity.name)
                elif entity.type == 'Obstacle':
                    xs = [entity.x + entity.r * math.cos(math.pi * 2.0 * i / 1000) for i in range(0, 1000)]
                    ys = [entity.y + entity.r * math.sin(math.pi * 2.0 * i / 1000) for i in range(0, 1000)]
                    plt.plot(xs, ys, c='k')

                elif entity.type == 'Gate':
                    plt.scatter(entity.x1, entity.y1, c='k')
                    plt.scatter(entity.x2, entity.y2, c='k')
                else:
                    print (entity.type, entity.name)
        
        if type == 'B' or type == 'C':
            cnt = 0
            for entity in self.points:
                if entity.name == 'Target0':
                    plt.scatter(entity.x, entity.y, c='r')
                elif entity.name == 'Target1':
                    plt.scatter(entity.x, entity.y, c='g')
                elif entity.name == 'Target2':
                    plt.scatter(entity.x, entity.y, c='b')
                else:
                    plt.scatter(entity.x, entity.y, c='k', label = 'P'+str(cnt))
                    cnt += 1

        if type == 'D':
            for entity in self.points:
                if 'Gate' in entity.name:
                    plt.scatter(entity.x, entity.y, c='k')

        if type == 'B' or type == 'C' or type == 'D':
            for entity in self.obstacles:
                xs = [entity.x + entity.r * math.cos(math.pi * 2.0 * i / 1000) for i in range(0, 1000)]
                ys = [entity.y + entity.r * math.sin(math.pi * 2.0 * i / 1000) for i in range(0, 1000)]
                plt.plot(xs, ys, c='k')

        if type == 'C' or type == 'D':
            xs = [item.x for item in self.answers]
            ys = [item.y for item in self.answers]
            plt.plot(xs, ys, c='y')

        plt.show()

    def build(self, divided = 10):
        self.obstacles = []
        self.points = []
        
        cnt = 0
        for entity in self.entities:
            if entity.type == 'Point':
                self.points.append(entity)
            elif entity.type == 'Obstacle':
                self.obstacles.append(entity)
            elif entity.type == 'Gate':
                self.points.append(Entity(type='Point', name='GateA'+str(cnt), x=entity.x1, y=entity.y1))
                self.points.append(Entity(type='Point', name='GateB'+str(cnt), x=entity.x2, y=entity.y2))
                cnt += 1
            else:
                print ('Error')

        self.minx = self.maxx = self.points[0].x
        self.miny = self.maxy = self.points[0].y 
        for point in self.points:
            self.minx = min(self.minx, point.x)
            self.miny = min(self.miny, point.y)
            self.maxx = max(self.maxx, point.x)
            self.maxy = max(self.maxy, point.y)

        for obstacle in self.obstacles:
            self.minx = min(self.minx, obstacle.x - obstacle.r)
            self.miny = min(self.miny, obstacle.y - obstacle.r)
            self.maxx = max(self.maxx, obstacle.x + obstacle.r)
            self.maxy = max(self.maxy, obstacle.y + obstacle.r)

        self.minx -= 2
        self.miny -= 2
        self.maxx += 2
        self.maxy += 2

        cnt = 0
        for i in range(divided+1):
            for j in range(divided+1):
                x = self.minx + (self.maxx - self.minx) * i / divided 
                y = self.miny + (self.maxy - self.miny) * j / divided 
                self.points.append(Entity(type='Point', name='Point'+str(cnt), x=x, y=y))

        newpoints = []
        for point in self.points:
            flag = True
            for obstacle in self.obstacles:
                if self.check_point_in_obstacle(point, obstacle):
                    flag = False
                    break
            if flag:
                newpoints.append(point)
        self.points = newpoints

    def search(self, targetnum=2, gatenum=4):
        name_to_entity = {}
        name_to_number = {}
        cnt = 0
        for point in self.points:
            name_to_entity[point.name] = point 
            name_to_number[point.name] = cnt 
            cnt += 1
            #print (point.name)
        
        heap = Heap()
        loc = name_to_number['Target0']
        record = Record(loc=loc, dis=0, gate=0, path=str(loc))
        heap.push(record)
        
        starttime = time.time()
        answer = None

        connect = {}
        for i in range(len(self.points)):
            for j in range(len(self.points)):
                flag = True 
                if i==j:
                    flag = False 
                else:
                    for obstacle in self.obstacles:
                        if self.check_insection_with_obstacle(self.points[i], self.points[j], obstacle):
                            flag = False 
                            break 
                connect [str(i) + '$' + str(j)] = flag

        while len(heap.heap):
            record = heap.pop()
            if heap.hash.get(record.hash) < record.dis:
                continue
            old_targetnum = record.gate % 10 
            old_gatenum = record.gate % 100 // 10
            old_gatevalue = record.gate // 100
            #print ('search ', record.gate, record.dis)
            #print ('\t\t', record.path)
            if old_targetnum == targetnum and old_gatenum == gatenum:
                answer = record 
                break
            for loc in range(len(self.points)):
                if loc == record.loc:
                    continue 
                if not connect[str(loc) + '$' + str(record.loc)]:
                    continue
                new_dis = record.dis + math.sqrt(self.distance_between_points(self.points[record.loc], self.points[loc]))
                new_path = record.path + '$' + str(loc)
                if self.points[loc].name == 'Target' + str(old_targetnum + 1):
                    new_targetnum = old_targetnum + 1
                    #print ('\t\t\ttarget', record.loc, loc, old_targetnum, self.points[loc].name, new_targetnum, new_dis)
                else:
                    new_targetnum = old_targetnum 
                
                name1 = self.points[record.loc].name
                name2 = self.points[loc].name
                new_gatenum = old_gatenum 
                new_gatevalue = old_gatevalue
                if 'Gate' in name1 and 'Gate' in name2:
                    if 'GateB' in name1:
                        name1, name2 = name2, name1 
                    if 'GateA' in name1 and 'GateB' in name2 and name1[5:] == name2[5:]:
                        number = 1<<int(name1[5:])
                        if number & old_gatevalue == 0:
                            new_gatenum += 1 
                            new_gatevalue |= number 

                new_record = Record(loc=loc, dis=new_dis, gate=new_gatevalue*100+new_gatenum*10+new_targetnum, path=new_path)
                heap.push(new_record)
        print ('Time ', time.time() - starttime)
        if answer is None:
            print ('No answer')
        else:
            print ('Answer')
            print (answer.dis)
            print (answer.path)
            self.answers = [self.points[int(item)] for item in answer.path.split('$')]
            print (len(self.answers))
            
            count = 0
            for point in self.answers:
                print ('\t', point.x, point.y, point.name)
                res, handle = vrep.simxGetObjectHandle(self.clientID, 'target'+str(count), vrep.simx_opmode_blocking)
                if point.name=='Target1':
                    res = vrep.simxSetObjectPosition(self.clientID, handle, -1, [point.x, point.y, 1], vrep.simx_opmode_blocking)
                elif point.name=='Target2':
                    res = vrep.simxSetObjectPosition(self.clientID, handle, -1, [point.x, point.y, 2], vrep.simx_opmode_blocking)
                elif point.name[0]=='G':
                    if point.name[4]=='A':
                        res = vrep.simxSetObjectPosition(self.clientID, handle, -1, [point.x, point.y, 3], vrep.simx_opmode_blocking)
                    else:
                        res = vrep.simxSetObjectPosition(self.clientID, handle, -1, [point.x, point.y, 4], vrep.simx_opmode_blocking)
                else:
                    res = vrep.simxSetObjectPosition(self.clientID, handle, -1, [point.x, point.y, 0], vrep.simx_opmode_blocking)
                count += 1

                

if __name__ == '__main__':
    search = Search([])
    point1 = Entity(type='Point', name='Tmp1', x=0, y=0)
    point2 = Entity(type='Point', name='Tmp2', x=10, y=0)
    obstacle = Entity(type='Obstacle', name='Tmp3', x=-4, y=3, r=4.9)
    print (search.check_insection_with_obstacle(point1, point2, obstacle))

