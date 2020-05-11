import numpy as np
import time
import shapely.geometry as geom
import matplotlib.pyplot as plt
import pygame


def vertices_from_state(state, collision_rectangle):
    center = np.array(state[:2])
    angle = state[2]
    size = np.array(collision_rectangle)

    vertices = np.array([[1.,1.], [1.,-1.],[-1.,-1.], [-1.,1.]])
    vertices = vertices*size/2

    R = np.array(([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]))
    vertices = np.dot(R, vertices.T).T
    vertices = vertices + center

    return vertices

def do_A_and_B_overlap(angleA, angleB, verticesA, verticesB):

    axis = []
    axis.append(np.array([np.cos(angleA), np.sin(angleA)]))
    axis.append(np.array([-np.sin(angleA), np.cos(angleA)]))
    axis.append(np.array([np.cos(angleB), np.sin(angleB)]))
    axis.append(np.array([-np.sin(angleB), np.cos(angleB)]))

    dots = []
    overlap_possible = True
    for ax in axis:
        dotA = np.dot(verticesA, ax)
        dotB = np.dot(verticesB, ax)
        if max(dotA) >= min(dotB) and max(dotB) >= min(dotA):
            overlap_possible = True
        else:
            overlap_possible = False
            break

    return overlap_possible

def create_road(total_turn_angle ,turn_point, initial_point, road_length = 100 ,resolution = 2.):

    total_turn_angle = -total_turn_angle * 3.14159 / 180.
    road_curvatures = np.concatenate([np.zeros(int(road_length*resolution*turn_point)),np.ones(10)*total_turn_angle/10.,np.zeros(int(road_length*resolution*(1-turn_point)))])

    vec = np.array([1,0])/resolution
    road_points = [initial_point]
    for c in road_curvatures:
        vec = np.dot(rotation_matrix(c), vec)
        #print(vec)
        road_points.append(road_points[-1]+ vec)
    road_points = np.array(road_points)
    return road_points
def find_index(min_ind, max_ind ,point, l):
    point = np.array(point)
    if max_ind - min_ind < 15:
        min_e = -1
        min_d = 100000
        for e in range(min_ind, max_ind):
            d = np.linalg.norm(l[e]-point)
            if d < min_d:
                min_d = d
                min_e = e
        return min_e
    split = 5
    min_e = -1
    min_d = 100000
    step = (max_ind - min_ind)//split
    for e in range(min_ind, max_ind, step):
        d = np.linalg.norm(l[e]-point)
        if d < min_d:
            min_d = d
            min_e = e

    return find_index(max(min_e - step, 0), min(min_e + step, len(l)), point, l)

def interpolate_points(points):
    max_distance = 1.
    for e in range(len(points)-1):
        p1 = points[e]
        p2 = points[e+1]
        d = np.linalg.norm(p2-p1)
        steps = int(d // max_distance) + 1
        alpha = np.linspace(0,1, steps)
        alpha = np.array([alpha, alpha]).T
        res = p1 * (1 -alpha) + p2 * alpha
        if e == 0:
            interpolated_points = res
        else:
            interpolated_points = np.concatenate([interpolated_points,res])
    return interpolated_points

def initialize_world():
    try:
        initial_point = np.array([-50,0.])
        maximum_roads = 1
        maximum_lanes = 3
        minimum_lanes = 2
        road_number = np.random.randint(1,maximum_roads+1)
        roads = []
        for e in range(road_number):

            points = create_road(np.random.randint(-90,90), np.random.randint(40,70)/100., initial_point)
            road = Road(points, lane_width = 4., lanes = np.random.randint(minimum_lanes,maximum_lanes+1))
            roads.append(road)
        #world = World(roads = [road1, road2])

        #world = World(roads = [road2], dt = 0.1)
        world = World(roads = roads, dt = 0.1)
        world.create_starting_points()
        world.initialize_cars()
        world.update_lane_map()
        return world
    except:
        return initialize_world()
def normalize(a):
    return (a -np.mean(a))/np.std(a)
def rotation_matrix(angle):
    return np.array(([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]))

class Game():
    def __init__(self, scale, width, height):
        self.gameDisplay = pygame.display.set_mode((width, height))
        self.gameDisplay.fill((255,255,255))
        self.clock = pygame.time.Clock()
        self.scale = scale
        self.width = width
        self.height = height

    def draw_roads(self,roads):
        #fill with white
        self.gameDisplay.fill((255,255,255))
        #draw roads
        for road in roads:
            #draw lanes
            for lane in road.lanes:
                pts = (lane * self.scale + np.array([self.width/2, self.height/2])).astype(int)
                pygame.draw.lines(self.gameDisplay, (200,200,200), False, pts.tolist(),  int(self.scale*(road.lane_width+1.)))
                pygame.draw.lines(self.gameDisplay, (255,255,255), False, pts.tolist())
    def draw_line(self, p1, p2, color):
        vertices = np.array([p1,p2])
        pygame.draw.lines(self.gameDisplay, color, False, (vertices*self.scale+np.array([self.width/2, self.height/2])).tolist())
    def draw_timeline(self, timesteps, current_t):
        ts = np.linspace(0.10,0.80, timesteps)
        h = self.height
        w = self.width
        top = int(h*0.93)
        width= int((ts[1] - ts[0])*w*0.5)
        height= int(width*2)
        filled=0
        for i,e in enumerate(ts):
            left = int(w*e)

            if i == current_t:
                pygame.draw.rect(self.gameDisplay, [255,0,0], [left, top, width*3, height*3], filled)

            else:
                pygame.draw.rect(self.gameDisplay, [100,100,100], [left, top, width, height], filled)

    def draw_dot(self,p):

        pos = (p * self.scale + np.array([self.width/2, self.height/2])).astype(int)
        pygame.draw.circle(self.gameDisplay, [200,0,0], pos, 4)

    def car(self, vertices, color):

        #x,y,psi,_ = state

        #gameDisplay.fill(white)
        #collision_im = pygame.Surface((int(collision_rectangle[0]*self.scale), (int(collision_rectangle[1]*self.scale))))
        #carImg = pygame.image.load('car.png')
        #carImg = pygame.transform.rotozoom(carImg, 0, 0.02)
        #carImg = pygame.transform.scale(carImg, collision_im.get_size())
        #collision_im.blit(carImg,(0,0))
        #pygame.draw.rect(collision_im,blue,(0,0) + collision_im.get_size(), 2)
        #collision_im = collision_im.convert_alpha()

        pygame.draw.polygon(self.gameDisplay, color, (vertices*self.scale+np.array([self.width/2, self.height/2])).tolist())

        #im = pygame.transform.rotozoom(collision_im, -psi*180./3.14159, 1.)
        #self.gameDisplay.blit(im, (x*self.scale+self.width/2.,y*self.scale+self.height/2))
    def display_stats(self, messages):
        black = (0, 0, 0)
        white = (255, 255, 255)
        font = pygame.font.Font('freesansbold.ttf', 12)
        h = 0.8
        for m in messages:
            text = font.render(m, True, black, white)
            textRect = text.get_rect()
            textRect.center = (int(self.width*0.25), int(self.height*h))
            self.gameDisplay.blit(text, textRect)
            h += 0.05
class Road():
    def __init__(self, points, lane_width, lanes):
        self.points = points
        self.lane_width = lane_width
        self.number_of_lanes = lanes
        self.lanes = []
        self.get_lanes()
        self.boundary = []
        self.get_road_boundary_polygon()
        self.lines = [geom.LineString(lane) for lane in self.lanes]


    def get_lanes(self):
        #pygame.draw.lines(self.gameDisplay, (100,100,100), False, show_road.tolist(), int(self.scale*road.lane_width*road.lanes))
        pts = self.points
        for lane in range(self.number_of_lanes):
            #pygame.draw.lines(self.gameDisplay, (100,100,100), False, pts.tolist(),  int(self.scale*road.lane_width+5))
            #pygame.draw.lines(self.gameDisplay, (200,255,200), False, pts.tolist())
            #poly = geom.Polygon(pts)
            # Create offset airfoils, both inward and outward
            #out_poly = poly.buffer(self.scale*road.lane_width/2)
            #pts = np.array(out_poly.exterior)
            #poly = geom.polygon.LinearRing(pts)
            pts = interpolate_points(pts)
            self.lanes.append(pts)
            poly = geom.LineString(pts)

            # Create offset airfoils, both inward and outward
            out_poly = poly.parallel_offset(self.lane_width, side="right", resolution=16,
                                                         join_style=2, mitre_limit=1)
            pts = np.array(out_poly)
            if pts[0][0] > pts[-1][0]:
                pts = pts[::-1]
    def get_road_boundary_polygon(self):
        left_side = np.array(geom.LineString(self.lanes[0]).parallel_offset(self.lane_width/2. + 1., side="left", resolution=16, join_style=2, mitre_limit=1))
        right_side = np.array(geom.LineString(self.lanes[-1]).parallel_offset(self.lane_width/2. + 1, side="right", resolution=16, join_style=2, mitre_limit=1))

        if left_side[0][0] > left_side[-1][0]:
            left_side = left_side[::-1]
        if right_side[0][0] > right_side[-1][0]:
            right_side = right_side[::-1]

        boundary = np.concatenate([left_side, right_side[::-1], np.array([left_side[0]])])
        self.boundary = geom.Polygon(boundary)

    def point_within_road(self, p):
        return geom.Point(p).within(self.boundary)

    def project_point(self, point, lane_number):
        point_on_line = self.lines[lane_number].interpolate(self.lines[lane_number].project(geom.Point(point[0], point[1])))
        return [point_on_line.x, point_on_line.y]
    def get_heading(self, point, lane_number):
        l = np.array(self.lines[lane_number])
        i = find_index(0, len(l) ,point, l)
        if i == len(l)-1:
            return l[i] - l[i-1]
        return l[i+1] - l[i]



class World():
    def __init__(self, roads, dt):
        self.total_agents_created = 0
        self.roads = roads
        self.agents = []
        self.dt = dt
        self.t = 0
        self.generate_road_boundary()
        self.laser_number = 36
        self.laser_max_range = 20.
        self.lane_map = [[{} for lane in range(road.number_of_lanes)] for road in self.roads]
        self.ai_created = False
        self.end_experiment = False
        self.completion_percentage = 0
        self.laser_readings = 0

        self.starting_points_and_lanes = None

        self.laser_remove_norm = np.array([4.        , 4.06533155, 4.27243219, 4.65956738, 3.79974594,
       3.19762002, 2.83898783, 2.62865556, 2.52283223, 2.50251987,
       2.56429216, 2.71878683, 2.99554292, 3.45879602, 4.25325404,
       4.43966506, 4.14953746, 4.01616791, 4.01616791, 4.14953746,
       4.43966506, 4.25325404, 3.45879602, 2.99554292, 2.71878683,
       2.56429216, 2.50251987, 2.52283223, 2.62865556, 2.83898783,
       3.19762002, 3.79974594, 4.65956738, 4.27243219, 4.06533155,
       4.        ])/2.

    def create_starting_points(self):

        starting_points = [[[] for lane in road.lanes] for road in self.roads]
        how_many_cars_in_lane = 5
        car_standard_spacing = 8.
        minimum_distance = 6.


        if len(self.roads) == 1:

            road = self.roads[0]
            number_of_lanes = road.number_of_lanes
             # compute average spacing between lane points
            points_in_10_meters = []
            for lane in road.lanes:
                i1 = int(len(lane)*0.05)
                p1 = lane[i1]

                i2 = int(len(lane)*0.35)
                p2 = lane[i2]

                d = np.linalg.norm(p2-p1)
                r = (i2 - i1)/d*10.
                points_in_10_meters.append(r)

            #print(points_in_10_meters)

            for lane_id ,lane in enumerate(road.lanes):
                how_far = 0.3 +np.random.randint(-10,10)/100.
                index = int(len(lane)*how_far)
                point = lane[index]
                starting_points[0][lane_id].append((index,point))
                game.draw_dot(point)

                sign = +1
                for e in range(how_many_cars_in_lane-1):
                    index  = index + sign * int((e+1)*points_in_10_meters[lane_id]*car_standard_spacing/10.)
                    point = lane[index]


                    #insert point if valid

                    prev = ([(0,0)] + starting_points[0][lane_id])[-sign][1]
                    if np.linalg.norm(point - prev) > minimum_distance:
                        if sign == 1:
                            starting_points[0][lane_id].append((index,point))
                        else:
                            starting_points[0][lane_id] = [(index,point)] + starting_points[0][lane_id]
                        game.draw_dot(point)
                    sign *= -1
            self.starting_points_and_lanes = starting_points

        else:
            print("Too many roads")

    def generate_road_boundary(self):
        union = geom.Polygon()
        for road in self.roads:
            union = union.union(road.boundary)
        self.total_boundary = union
    def is_agent_out_of_track(self, vertices):
        res = False
        for v in vertices:
            is_on_road = False
            for road in self.roads:
                if road.point_within_road(v):
                    is_on_road = True
            if not is_on_road:
                return True
        return res

    def check_ai_crash(self):
        for agent in self.agents:
            if agent.agent_id == 0:
                ai_vertices = agent.vertices_polygon
                for v in agent.vertices:
                    if not geom.Point(v).within(self.total_boundary):
                        return True
        for agent in self.agents:
            if agent.agent_id != 0:
                if agent.vertices_polygon.intersects(ai_vertices):
                    return True
        return False

    def get_laser_readings(self):
        # find laser candidates
        laser_number = self.laser_number
        laser_angles = np.linspace(0, 2*np.pi, laser_number)
        for agent in self.agents:
            if agent.agent_id == 0:
                xc,yc,psic,vc = agent.state

        vectors = np.array([xc,yc]) + self.laser_max_range*np.array([np.cos(laser_angles+psic), np.sin(laser_angles+psic)]).T

        # road boundary
        free_road = self.total_boundary
        for agent in self.agents:
            if agent.agent_id != 0:
                free_road = free_road.difference(agent.vertices_polygon)

        # laser lines
        lines = [[[xc,yc],p.tolist()] for p in vectors]
        intersections = []
        for l in lines:
            intersection = free_road.intersection(geom.LineString(l))

            if intersection.type == 'LineString':
                intersection = np.array(intersection)[-1]
            elif intersection.type == 'MultiLineString':
                intersection = np.array(intersection[0])[-1]
            #    intersection = np.array(intersection.boundary)[0].tolist()
            else:
                print('ERROR')
            #    intersection = np.array(intersection).tolist()

            intersections.append(intersection)
        laser_readings = np.linalg.norm(np.array(intersections) - np.array([xc,yc]), axis = 1)
        return intersections, laser_readings
    def update_lane_map(self):

        self.lane_map = [[{} for lane in range(road.number_of_lanes)] for road in self.roads]
        for agent in self.agents:
            road = agent.control.desired_road
            lane = agent.control.desired_lane
            x,y,psi,v = agent.state
            point = [x,y]
            #print('A:')
            #print(len(self.roads))
            #print(road)
            #print(lane)
            l = np.array(self.roads[road].lines[lane])
            i = find_index(0, len(l) ,point, l)
            if agent.agent_id == 0:
                self.completion_percentage = float(i) / (0.7*len(l))

            if  float(i) / (0.7*len(l)) > 1.1:

                self.agents.remove(agent)
            else:
                self.lane_map[road][lane][agent.agent_id] = i
    def place_car(self, road, lane, point, is_ai):

        x,y = point + np.random.randn(2)/100.
        psi = np.random.randn(1)[0] / 25.
        v = 3.5 + np.random.randn(1)[0]
        state = np.array([x,y,psi,v])

        if is_ai:
            color = [0,0,255]
            agent_id = 0
            collision_rectangle = np.array([4,2.5])
            self.ai_created = is_ai

        else:
            color = [0,200,0]
            agent_id = self.total_agents_created
            collision_rectangle = (np.array([4,2.5]) + np.random.randn(2)/ 10.)*(1+np.random.randn(1)[0]*0.1)

        agent = Agent(agent_id, state, collision_rectangle, self.dt)

        agent.control.desired_road = 0
        agent.control.desired_lane = lane
        agent.color = color
        agent.control.is_ai = is_ai

        self.agents.append(agent)
        self.total_agents_created += 1


    def initialize_cars(self):

        place_car_probability = 0.5

        # decide where to place AI car
        n = self.roads[0].number_of_lanes
        assigned_lane = np.random.randint(n)
        m = len(self.starting_points_and_lanes[0][assigned_lane])
        lane_place = 1 +np.random.randint(m-2)
        index, point = self.starting_points_and_lanes[0][assigned_lane][lane_place]

        self.starting_points_and_lanes[0][assigned_lane].remove((index, point))

        self.place_car(0, assigned_lane, point, True)
        #print(self.ai_created)
        #place other cars

        for i in range(n):
            while len(self.starting_points_and_lanes[0][i]) > 0:
                if np.random.binomial(1,place_car_probability):
                    index, point = self.starting_points_and_lanes[0][i][0]
                    self.place_car(0, i, point, False)
                self.starting_points_and_lanes[0][i] = self.starting_points_and_lanes[0][i][1:]


    def generate_agent(self, ai = False):
        starting_id = 20
        minimum_distance = 20.
        lane_candidates = []
        max_lanes = max([road.number_of_lanes for road in self.roads])
        joint_lanes_min_id = [1000 for e in range(max_lanes)]

        for road in self.lane_map:
            for lane_number, lane in enumerate(road):
                if len(lane) > 0:
                    min_id = min([a for a in lane.values()])
                    if joint_lanes_min_id[lane_number] > min_id:
                        joint_lanes_min_id[lane_number] = min_id

        for road_number, road in enumerate(self.lane_map):
            for lane_number, lane in enumerate(road):
                if joint_lanes_min_id[lane_number] - starting_id >= minimum_distance:
                    lane_candidates.append((road_number,lane_number))

        if len(lane_candidates) > 0:
            (road,lane) = lane_candidates[np.random.randint(len(lane_candidates))]
            x,y = self.roads[road].lanes[lane][starting_id] + np.random.randn(2)
            psi = np.random.randn(1)[0] / 5
            v = 3.5 + np.random.randn(1)[0]
            state = np.array([x,y,psi,v])
            self.total_agents_created += 1
            if ai:
                x,y = self.roads[road].lanes[lane][starting_id] + np.random.randn(4)/100.
                psi = np.random.randn(1)[0] / 25.
                color = [0,0,255]
                agent_id = 0
                collision_rectangle = np.array([4,2.5])
                agent = Agent(agent_id, state, collision_rectangle, self.dt)
                agent.control.desired_road = road
                agent.control.desired_lane = lane
                #color[road] += 150
                #color[lane] += 80.
                agent.color = color
                self.ai_created = True
                agent.control.is_ai = True

            else:
                color = [0,255,0]

                agent_id = self.total_agents_created
                collision_rectangle = (np.array([4,2.5]) + np.random.randn(2)/ 10.)*(1+np.random.randn(1)[0]*0.1)
                agent = Agent(agent_id, state, collision_rectangle, self.dt)
                agent.control.desired_road = road
                agent.control.desired_lane = lane
                #color[road] += 150
                #color[lane] += 80.
                agent.color = color
            self.agents.append(agent)



    def get_agent_lane_pairs(self):
        agent_lane_pairs = []
        agent_indexes = {}
        for agent_index, agent in enumerate(self.agents):
            agent_indexes[agent.agent_id] = agent_index

        for road_number, road in enumerate(self.lane_map):
            for lane_number, lane in enumerate(road):
                for agent_id in lane:
                    lane_point = self.roads[road_number].lanes[lane_number][lane[agent_id]]
                    x,y,psi,v = self.agents[agent_indexes[agent_id]].state
                    agent_lane_pairs.append([np.array([x,y]), lane_point])
        return agent_lane_pairs

    def get_stats(self):
        messages = []
        messages.append('Number of agents: {}'.format(len(self.agents)))
        messages.append('Completion percentage: {}%'.format(np.round(self.completion_percentage*100, 2)))
        if self.ai_created:
            for agent in self.agents:
                if agent.control.is_ai:
                    a = agent.control.execute_action
                    messages.append('Action: {}'.format(np.round(a,1)))

        return messages
    def step_before_AI_created(self):
        global game, AI
        # (visuals)draw roads
        visualize = True
        if visualize: game.draw_roads(self.roads)

        #t1 = time.time()

        # generate agents if possible
        self.generate_agent()

        #t11 = time.time()


        #t12 = time.time()

        # generate AI agent
        #if self.t > 2. and not self.ai_created:
        #    self.generate_agent(ai = True)
        #    self.update_lane_map()

        #t2 = time.time()

        # obtain agent actions and perform step
        for agent in self.agents:
            u0 = agent.control.select_driving_action(agent.state, agent.action, self)
            agent.action = u0
            x0 = agent.state
            agent.update_state(agent.control.get_next_state(agent.state,u0))
            if visualize: game.car(agent.vertices, agent.color)

        self.update_lane_map()

        #t3 = time.time()


        # (visuals)display map
        if visualize:
            game.display_stats(self.get_stats())
            pygame.display.update()
        # time update
        self.t += self.dt

        #ts = [t1,t11,t12,t2,t3]
        #ms = ['    Generate Agent', '   Update map', 'AI gen',  'Select control actions and update states']

        #t = t3-t1
        #print('BF BREAKDOWN:')
        #for i,m in enumerate(ms):
        #    print('   - '+m+': {}'.format(np.round((ts[i+1]-ts[i])/t*100, 1)))

    def step_after_AI_created(self):
        # perform movement
        #t1 = time.time()
        self.step_before_AI_created()
        #t2 = time.time()


        # check agent crashes
        ai_crashed = self.check_ai_crash()
        if ai_crashed:
            AI.crashed = True
            self.end_experiment = True
        #t3 = time.time()
        # process laser readings
        intersections, laser_readings = self.get_laser_readings()
        self.laser_readings = laser_readings

        #t4 = time.time()
        # get agent position
        for agent in self.agents:
            if agent.agent_id == 0:
                x,y,psi,v = agent.state
                vertices = agent.vertices
                color = agent.color
                action = agent.action
        # show lasers
        #ii = 0
        #for laser_point in intersections:
        #    ii+=1
        #    p1 = [x,y]
        #    p2 = laser_point
        #    game.draw_line(p1,p2, (30,30,30))
        #    if ii>2:
        #        break
        game.car(vertices, color)

        state = np.array(laser_readings) - self.laser_remove_norm
        state = 10./(state + 0.5)
        #if np.sum(ls <= 0.)>0:
        #    AI.crashed = True
        #    state = np.zeros(self.laser_number)
        #else:
        #    state = -np.log(0.05 +ls)+1.

        state[-1] = v
        # action
        action = action

        # reward
        if AI.crashed:
            if self.completion_percentage > 0.0:
                reward = -4.
            else:
                reward = 0.
            self.end_experiment = True
        elif self.completion_percentage > 1:
            reward = +0.
            self.end_experiment = True
        else:
            reward = 0.8*(self.completion_percentage -  AI.completion_percentage)
            AI.completion_percentage = self.completion_percentage

        # state
        #state = 2./(0.1 + np.array(laser_readings))



        #AI.memory.add_to_memory(state, action, reward)

        #t6 = time.time()
        #self.create_starting_points()

        pygame.display.update()

        #t7 = time.time()

        #ts = [t1,t2,t3,t4,t5,t6,t7]
        #ms = ['Kinematic step', 'Check crashes', 'Get laser readings', 'Get agent position', 'Rewards', 'Pygame Display']

        #t = t7-t1
        #for i,m in enumerate(ms):
        #    print(m+': {}'.format(np.round((ts[i+1]-ts[i])/t*100, 1)))


        return state, reward



class Control():
    def __init__(self, collision_rectangle, dt, mode):
        # car parameters
        self.collision_rectangle  = collision_rectangle
        car_length = collision_rectangle[0]
        self.lf = car_length*(0.65)/2
        self.lr = car_length*(0.65)/2
        self.dt = dt
        # control variables
        self.desired_road = 0
        self.desired_lane = 0
        self.desired_direction = 1
        self.delta_rate = 0.25
        self.max_delta = 0.8
        self.ey_history = [0,0,0,0,0]
        self.mode = mode
        self.is_ai = False
        self.execute_action = [0.,0.]

    def select_driving_action(self, state, old_action, world):

        # rarity: change desired lane
        if not self.is_ai:
            change_lane_frequency = 1./100. # 1 time every 100 timesteps on average
            if np.random.randint(1,int(1./change_lane_frequency)) == 10:
                lane_number = world.roads[self.desired_road].number_of_lanes
                current_lane = self.desired_lane
                if lane_number == 1:
                    new_lane = current_lane

                else:
                    if current_lane == 0:
                        new_lane = 1
                    elif current_lane == lane_number-1:
                        new_lane = current_lane-1
                    else:
                        new_lane = current_lane + np.random.choice([-1,+1])
                self.desired_lane = new_lane

        old_delta = old_action[0]
        if self.mode == 'PID':
            # PID
            kp = 0.2
            kd = 0.9
            ki = 0.2
            old_ey = self.ey_history[-1]

            ey, e_heading = self.position_error(state, world) * self.desired_direction
            self.ey_history =  self.ey_history[1:] + [ey]
            I = ki * np.sum(self.ey_history)/len(self.ey_history)
            delta = -kp*ey - kd*(ey - old_ey)/self.dt - I
            delta = np.clip(delta, old_delta-self.delta_rate, old_delta+self.delta_rate)
            delta = np.clip(delta, -self.max_delta, self.max_delta)
            action = old_action
            action[0] = delta

            if self.is_ai:
                return self.execute_action

            return action
        if self.mode == 'simple_MPC':
            timestep_horizon = 3
            Ky = 1/2.**2
            Kang = 1/0.9**2


            delta_min = max(-self.max_delta, old_delta -self.delta_rate)
            delta_max = min(self.max_delta, old_delta + self.delta_rate)
            delta_candidates = np.linspace(delta_min, delta_max, 8)
            min_error = 100000
            min_delta = old_delta
            #print('AAAAAAAAAAAA')
            for delta in delta_candidates:
                #print('Delta: {}'.format(delta))
                error = 0
                action = [delta, 0]
                s = state
                for t in range(timestep_horizon):
                    #print('State {} = {}'.format(t, s))
                    s = self.get_next_state(s, action)
                    ey, e_heading = self.position_error(s, world)

                    error += Ky*ey**2 + Kang*e_heading**2
                    #print('State {} = {}, Ey = {}'.format(t, s, ey))
                #print(delta, error)
                if error < min_error:
                    min_error = error
                    min_delta = delta

            action = old_action
            action[0] = min_delta
            return action



    def position_error(self, state, world):

        x,y,psi,v = state

        point_on_line = world.roads[self.desired_road].project_point([x,y], self.desired_lane)
        road_heading = world.roads[self.desired_road].get_heading([x,y], self.desired_lane)
        psid = np.arctan2(road_heading[1], road_heading[0])
        xd, yd = point_on_line
        ey = -(x-xd)*np.sin(psid) + (y-yd)*np.cos(psid)
        #return np.linalg.norm([x-xd,y-yd])
        e_heading = min((psid -psi)%(2*np.pi), (psi -psid)%(2*np.pi))

        return ey, e_heading

    def get_next_state(self,x0,u0):
        x,y,psi,v = x0
        delta, ax = u0
        lf, lr, dt = [self.lf, self.lr, self.dt]

        beta = np.arctan(lr/(lf+lr)*np.tan(delta))
        x_n = x + dt*(v * np.cos(psi+beta))
        y_n = y + dt*(v * np.sin(psi+beta))
        psi_n = psi + dt*(v / lr * np.sin(beta))
        v_n = v + dt*ax

        state = np.array([x_n, y_n, psi_n, v_n])
        return state



class Agent():
    def __init__(self, agent_id, initial_state, collision_rectangle, dt):
        self.agent_id = agent_id
        self.state = initial_state
        self.action = [0,0]
        self.vertices = vertices_from_state(self.state, collision_rectangle)
        self.vertices_polygon = geom.Polygon(self.vertices)
        self.crashed = False
        self.collision_rectangle = collision_rectangle
        self.color = (0,0,255)

        # control
        #self.control = Control(collision_rectangle, dt, 'simple_MPC')
        self.control = Control(collision_rectangle, dt, 'PID')
    def update_state(self,state):
        self.state = state
        self.vertices = vertices_from_state(state, self.collision_rectangle)
        self.vertices_polygon = geom.Polygon(self.vertices)



class AI_class:
    def __init__(self):
        # EXPERIENCE COLLECTION
        self.memory = Memory()
        self.completion_percentage = -1
        self.crashed = False

        # DL MODEL

    def reset_experiment(self):
        self.completion_percentage = -1
        self.crashed = False
class Memory:
    def __init__(self):
        self.experiments = []

    def initialize_experiment(self):
        self.experiments.append(Experiment())

    def add_to_memory(self,state, action, reward):
        self.experiments[-1].rewards.append(reward)
        #self.experiments[-1].states.append(1./(0.2+state))
        self.experiments[-1].states.append(state)
        self.experiments[-1].actions.append(action)
class Experiment:
    def __init__(self):
        self.rewards = []
        self.states = []
        self.actions = []
class CarsEnvironment():
    def __init__(self):
        global AI,game

        self.observation_space = np.zeros(36*2)
        self.action_space = ActionSpace(2,1)

        width = 800
        height = 600
        scale = 10
        white = (255,255,255)
        game = Game(scale, width, height)
        pygame.init()
        AI = AI_class()
        self.world = None
        self.max_action = np.array([0.5,3.])
        self.previous_state = None
        self.reset()

    def reset(self):
        global AI
        AI.reset_experiment()
        self.world = initialize_world()
        try:

            #print(self.world.end_experiment)
            while not self.world.ai_created:
                self.world.step_before_AI_created()

            AI.memory.initialize_experiment()
            AI.completion_percentage = self.world.completion_percentage
            #print(self.world.completion_percentage)
            state, reward = self.world.step_after_AI_created()

        except:
            state = np.zeros(36)

        return self.processed_state(state, first = True)
    def step(self,action):
        global frame, game

        #pygame.image.save(game.gameDisplay, 'capturasnoob/'+str(frame).zfill(12)+'.jpeg')
        frame += 1
        try:
            for agent in self.world.agents:
                if agent.control.is_ai:

                    agent.control.execute_action = np.array(action) * self.max_action
            #print(self.world.end_experiment)
            state, reward = self.world.step_after_AI_created()
            mstate = self.processed_state(state, first = False)


            #print(self.world.end_experiment)
            return mstate,reward, self.world.end_experiment, []

        except:
            return np.zeros(72), -1, True, []
    def processed_state(self,state, first = False):

        if first:
            res = merge_states(state, state)
        else:
            res = merge_states(state, self.previous_state)

        self.previous_state = state

        #print(res)

        return res





def merge_states(s1,s2):
    return np.concatenate([s1/10.,s1-s2])

class ActionSpace():
    def __init__(self,action_dim, high=1):
        self.shape = [action_dim]
        self.high = [high]
    def sample(self):
        dim = self.shape[0]
        high = self.high[0]
        return np.random.randn(dim)*high
