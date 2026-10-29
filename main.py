import argparse
import math
import os
import random
import sys
from collections import deque

if "--smoke-test" in sys.argv:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame


SCREEN_WIDTH = 900
SCREEN_HEIGHT = 700
ROAD_WIDTH = 220
CENTER_X = SCREEN_WIDTH // 2
CENTER_Y = SCREEN_HEIGHT // 2
CAR_LENGTH = 42
CAR_WIDTH = 24
CAR_GAP = 18
MIN_CAR_CENTER_DISTANCE = CAR_LENGTH + CAR_GAP
FOLLOWING_TIME_HEADWAY = 1.0
STOP_DWELL_SECONDS = 1.0
DECELERATION = 220.0
ACCELERATION = 120.0
PERMISSIVE_CONFLICT_DISTANCE = 115.0
INTERSECTION_HALF_SIZE = ROAD_WIDTH / 2 + 12
CROSSWALK_DEPTH = 34
CROSSWALK_GAP = 8
CROSSWALK_CENTER_OFFSET = CROSSWALK_DEPTH / 2 + CROSSWALK_GAP
STOP_BAR_OFFSET = (
    CROSSWALK_CENTER_OFFSET + CROSSWALK_DEPTH / 2
    + CROSSWALK_GAP + CAR_LENGTH / 2
)
PEDESTRIAN_WALK_DURATION = 8.0
PEDESTRIAN_CLEARANCE_DURATION = 3.0
PEDESTRIAN_SPEED = 82.0
PEDESTRIAN_SPAWN_INTERVAL = 0.9
PEDESTRIAN_COLLISION_BUFFER = 8.0
WALK_VEHICLE_AXIS = "north_south"
WALK_CROSSWALKS_BY_AXIS = {
    "north_south": ("east", "west"),
    "east_west": ("north", "south"),
}
FIXED_PHASE_DURATIONS = {
    "north_south_protected_left": 4.0,
    "north_south_permissive": 10.0,
    "north_south_yellow": 2.0,
    "east_west_protected_left": 4.0,
    "east_west_permissive": 10.0,
    "east_west_yellow": 2.0,
    "pedestrian_walk": PEDESTRIAN_WALK_DURATION,
    "pedestrian_clearance": PEDESTRIAN_CLEARANCE_DURATION,
}

BACKGROUND = (47, 105, 61)
ROAD = (52, 55, 59)
ROAD_EDGE = (225, 225, 210)
LANE_MARK = (205, 200, 165)
WHITE = (245, 245, 235)
TEXT = (240, 240, 240)
RED = (226, 65, 65)
YELLOW = (245, 195, 55)
GREEN = (65, 200, 100)
CAR_COLORS = [
    (70, 155, 235),
    (235, 115, 75),
    (230, 205, 85),
    (175, 115, 210),
    (225, 225, 225),
    (65, 190, 180),
]
CAR_SPRITES = {}
ROTATED_CAR_SPRITES = {}

DIRECTIONS = ("north", "south", "east", "west")
APPROACH_VECTORS = {
    "north": (0, 1),
    "south": (0, -1),
    "east": (-1, 0),
    "west": (1, 0),
}
LEFT_TURN_EXITS = {
    "north": "east",
    "south": "west",
    "east": "south",
    "west": "north",
}
RIGHT_TURN_EXITS = {
    "north": "west",
    "south": "east",
    "east": "north",
    "west": "south",
}
OPPOSITE_APPROACH = {
    "north": "south",
    "south": "north",
    "east": "west",
    "west": "east",
}

# Three separate approach lanes, ordered from the vehicle's right to left.
LANES = {
    "north": {
        "right": (CENTER_X - 94, None),
        "through": (CENTER_X - 58, None),
        "left": (CENTER_X - 22, None),
    },
    "south": {
        "right": (CENTER_X + 94, None),
        "through": (CENTER_X + 58, None),
        "left": (CENTER_X + 22, None),
    },
    "east": {
        "right": (None, CENTER_Y - 94),
        "through": (None, CENTER_Y - 58),
        "left": (None, CENTER_Y - 22),
    },
    "west": {
        "right": (None, CENTER_Y + 94),
        "through": (None, CENTER_Y + 58),
        "left": (None, CENTER_Y + 22),
    },
}
STOP_POINTS = {
    "north": (CENTER_X, CENTER_Y - ROAD_WIDTH / 2 - STOP_BAR_OFFSET),
    "south": (CENTER_X, CENTER_Y + ROAD_WIDTH / 2 + STOP_BAR_OFFSET),
    "east": (CENTER_X + ROAD_WIDTH / 2 + STOP_BAR_OFFSET, CENTER_Y),
    "west": (CENTER_X - ROAD_WIDTH / 2 - STOP_BAR_OFFSET, CENTER_Y),
}
CROSSWALKS = {
    "north": {
        "start": (CENTER_X - ROAD_WIDTH / 2 - 18, CENTER_Y - ROAD_WIDTH / 2 - CROSSWALK_CENTER_OFFSET),
        "end": (CENTER_X + ROAD_WIDTH / 2 + 18, CENTER_Y - ROAD_WIDTH / 2 - CROSSWALK_CENTER_OFFSET),
        "orientation": "horizontal",
    },
    "south": {
        "start": (CENTER_X - ROAD_WIDTH / 2 - 18, CENTER_Y + ROAD_WIDTH / 2 + CROSSWALK_CENTER_OFFSET),
        "end": (CENTER_X + ROAD_WIDTH / 2 + 18, CENTER_Y + ROAD_WIDTH / 2 + CROSSWALK_CENTER_OFFSET),
        "orientation": "horizontal",
    },
    "east": {
        "start": (CENTER_X + ROAD_WIDTH / 2 + CROSSWALK_CENTER_OFFSET, CENTER_Y - ROAD_WIDTH / 2 - 18),
        "end": (CENTER_X + ROAD_WIDTH / 2 + CROSSWALK_CENTER_OFFSET, CENTER_Y + ROAD_WIDTH / 2 + 18),
        "orientation": "vertical",
    },
    "west": {
        "start": (CENTER_X - ROAD_WIDTH / 2 - CROSSWALK_CENTER_OFFSET, CENTER_Y - ROAD_WIDTH / 2 - 18),
        "end": (CENTER_X - ROAD_WIDTH / 2 - CROSSWALK_CENTER_OFFSET, CENTER_Y + ROAD_WIDTH / 2 + 18),
        "orientation": "vertical",
    },
}


def dot(point, vector):
    return point[0] * vector[0] + point[1] * vector[1]


def car_collision_axes(car):
    if car.state == "turning" and car.turn_curve:
        p0, p1, p2, p3 = car.turn_curve
        t = min(1.0, max(0.0, car.turn_elapsed / car.turn_duration))
        u = 1.0 - t
        heading = (
            3 * u ** 2 * (p1[0] - p0[0])
            + 6 * u * t * (p2[0] - p1[0])
            + 3 * t ** 2 * (p3[0] - p2[0]),
            3 * u ** 2 * (p1[1] - p0[1])
            + 6 * u * t * (p2[1] - p1[1])
            + 3 * t ** 2 * (p3[1] - p2[1]),
        )
    else:
        heading = car.travel_vector
    length = math.hypot(*heading)
    if length <= 1e-9:
        heading = car.travel_vector
        length = math.hypot(*heading)
    forward = (heading[0] / length, heading[1] / length)
    return forward, (-forward[1], forward[0])


def car_outbound_vector(car):
    if car.state == "turning" and car.turn_exit:
        return APPROACH_VECTORS[OPPOSITE_APPROACH[car.turn_exit]]
    if car.state == "departing":
        return car.travel_vector
    return None


def car_collision_boxes_overlap(first, second, include_safe_gap=True):
    first_axes = car_collision_axes(first)
    second_axes = car_collision_axes(second)
    center_delta = (second.x - first.x, second.y - first.y)
    half_length = CAR_LENGTH / 2
    if include_safe_gap:
        half_length += CAR_GAP / 2
    first_half_length = half_length
    second_half_length = first_half_length
    half_width = CAR_WIDTH / 2

    for axis in (*first_axes, *second_axes):
        center_distance = abs(dot(center_delta, axis))
        first_radius = (
            first_half_length * abs(dot(first_axes[0], axis))
            + half_width * abs(dot(first_axes[1], axis))
        )
        second_radius = (
            second_half_length * abs(dot(second_axes[0], axis))
            + half_width * abs(dot(second_axes[1], axis))
        )
        if center_distance >= first_radius + second_radius - 1e-6:
            return False
    return True


def car_overlaps_pedestrian(car, pedestrian, position=None):
    axes = car_collision_axes(car)
    car_x, car_y = position or (car.x, car.y)
    pedestrian_x, pedestrian_y = pedestrian.position
    offset = (pedestrian_x - car_x, pedestrian_y - car_y)
    longitudinal = abs(dot(offset, axes[0]))
    lateral = abs(dot(offset, axes[1]))
    return (
        longitudinal
        < CAR_LENGTH / 2 + PEDESTRIAN_COLLISION_BUFFER
        and lateral
        < CAR_WIDTH / 2 + PEDESTRIAN_COLLISION_BUFFER
    )


def car_sprite(color):
    if color not in CAR_SPRITES:
        sprite = pygame.Surface((CAR_LENGTH, CAR_WIDTH), pygame.SRCALPHA)
        pygame.draw.rect(sprite, color, (0, 0, CAR_LENGTH, CAR_WIDTH),
                         border_radius=6)
        pygame.draw.rect(sprite, (35, 38, 42), (0, 0, CAR_LENGTH, CAR_WIDTH),
                         width=2, border_radius=6)
        pygame.draw.rect(sprite, (45, 60, 72), (13, 4, 16, CAR_WIDTH - 8),
                         border_radius=3)
        CAR_SPRITES[color] = sprite
    return CAR_SPRITES[color]


def rotated_car_sprite(color, angle):
    key = (color, angle)
    if key not in ROTATED_CAR_SPRITES:
        ROTATED_CAR_SPRITES[key] = pygame.transform.rotate(
            car_sprite(color), angle
        )
    return ROTATED_CAR_SPRITES[key]


def create_turn_curve(direction, exit_name, start):
    vx, vy = APPROACH_VECTORS[direction]
    out_approach = OPPOSITE_APPROACH[exit_name]
    out_x, out_y = APPROACH_VECTORS[out_approach]
    end_lane_x, end_lane_y = LANES[out_approach]["through"]
    if out_x:
        road_edge = CENTER_X + ROAD_WIDTH / 2 if out_x > 0 else CENTER_X - ROAD_WIDTH / 2
        end = (road_edge + out_x * 18, float(end_lane_y))
    else:
        road_edge = CENTER_Y + ROAD_WIDTH / 2 if out_y > 0 else CENTER_Y - ROAD_WIDTH / 2
        end = (float(end_lane_x), road_edge + out_y * 18)
    curve_size = 82 if exit_name == LEFT_TURN_EXITS[direction] else 43
    first_control = (start[0] + vx * curve_size, start[1] + vy * curve_size)
    second_control = (end[0] - out_x * curve_size, end[1] - out_y * curve_size)
    return start, first_control, second_control, end


class TrafficLight:
    PHASES = (
        "north_south_protected_left",
        "north_south_permissive",
        "north_south_yellow",
        "east_west_protected_left",
        "east_west_permissive",
        "east_west_yellow",
        "pedestrian_walk",
        "pedestrian_clearance",
    )
    MIN_GREEN_DURATIONS = {
        "north_south_protected_left": 4.0,
        "north_south_permissive": 10.0,
        "east_west_protected_left": 4.0,
        "east_west_permissive": 10.0,
    }
    MAX_GREEN_DURATIONS = {
        "north_south_protected_left": 8.0,
        "north_south_permissive": 18.0,
        "east_west_protected_left": 8.0,
        "east_west_permissive": 18.0,
    }
    YELLOW_DURATION = 2.0
    FIXED_DURATIONS = {
        "pedestrian_walk": PEDESTRIAN_WALK_DURATION,
        "pedestrian_clearance": PEDESTRIAN_CLEARANCE_DURATION,
    }

    def __init__(self):
        self.phase_index = 0
        self.elapsed = 0.0
        self.pedestrian_walk_axis = WALK_VEHICLE_AXIS
        self.visited_phases = {self.PHASES[0]}

    @property
    def phase(self):
        return self.PHASES[self.phase_index]

    @property
    def pedestrians_walk(self):
        return self.phase == "pedestrian_walk"

    @property
    def pedestrian_clearance(self):
        return self.phase == "pedestrian_clearance"

    @property
    def green_extension(self):
        minimum = self.MIN_GREEN_DURATIONS.get(self.phase)
        if minimum is None:
            return 0.0
        return max(0.0, self.elapsed - minimum)

    def walk_crosswalks(self):
        if self.pedestrians_walk:
            return WALK_CROSSWALKS_BY_AXIS[self.pedestrian_walk_axis]
        if self.phase == "north_south_permissive":
            return WALK_CROSSWALKS_BY_AXIS["north_south"]
        if self.phase == "east_west_permissive":
            return WALK_CROSSWALKS_BY_AXIS["east_west"]
        if self.phase.startswith("north_south"):
            return WALK_CROSSWALKS_BY_AXIS["east_west"]
        if self.phase.startswith("east_west"):
            return WALK_CROSSWALKS_BY_AXIS["north_south"]
        return WALK_CROSSWALKS_BY_AXIS[self.pedestrian_walk_axis]

    def active_axis_queues(self, queue_counts):
        if self.phase.startswith("north_south"):
            return (queue_counts["north"] + queue_counts["south"],
                    queue_counts["east"] + queue_counts["west"])
        if self.phase.startswith("east_west"):
            return (queue_counts["east"] + queue_counts["west"],
                    queue_counts["north"] + queue_counts["south"])
        return 0, 0

    def update(self, dt, queue_counts, pedestrian_ready=True):
        self.elapsed += dt
        while True:
            minimum_green = self.MIN_GREEN_DURATIONS.get(self.phase)
            if minimum_green is not None:
                active_queue, competing_queue = self.active_axis_queues(queue_counts)
                max_green = self.MAX_GREEN_DURATIONS[self.phase]
                if self.elapsed < minimum_green:
                    break
                if self.elapsed < max_green and active_queue > competing_queue:
                    break
                duration = min(max(self.elapsed, minimum_green), max_green)
            else:
                duration = self.FIXED_DURATIONS.get(
                    self.phase, self.YELLOW_DURATION
                )
                if self.elapsed < duration:
                    break

            self.elapsed = max(0.0, self.elapsed - duration)
            self.phase_index = (self.phase_index + 1) % len(self.PHASES)
            if self.pedestrians_walk:
                self.pedestrian_walk_axis = (
                    "east_west"
                    if self.pedestrian_walk_axis == "north_south"
                    else "north_south"
                )
            self.visited_phases.add(self.phase)
            if self.pedestrians_walk and not pedestrian_ready:
                self.elapsed = 0.0
                break
            if self.elapsed <= 0.0:
                break

    def left_color_for(self, direction):
        north_south = direction in ("north", "south")
        if self.pedestrian_clearance:
            return RED
        if self.pedestrians_walk:
            return (
                GREEN
                if north_south == (
                    self.pedestrian_walk_axis == "north_south"
                )
                else RED
            )
        if self.phase == "north_south_protected_left" and north_south:
            return GREEN
        if self.phase == "east_west_protected_left" and not north_south:
            return GREEN
        return RED

    def through_color_for(self, direction):
        north_south = direction in ("north", "south")
        if self.pedestrian_clearance:
            return RED
        if self.pedestrians_walk:
            walk_axis_matches = north_south == (
                self.pedestrian_walk_axis == "north_south"
            )
            return GREEN if walk_axis_matches else RED
        if self.phase == "north_south_permissive" and north_south:
            return GREEN
        if self.phase == "east_west_permissive" and not north_south:
            return GREEN
        if self.phase == "north_south_yellow" and north_south:
            return YELLOW
        if self.phase == "east_west_yellow" and not north_south:
            return YELLOW
        return RED

    def phase_label(self):
        labels = {
            "north_south_protected_left": "BAC-NAM: RE TRAI BAO VE (MUI TEN XANH)",
            "north_south_permissive": "BAC-NAM: XANH TRON / RE TRAI NHUONG DUONG",
            "north_south_yellow": "BAC-NAM: DEN VANG",
            "east_west_protected_left": "DONG-TAY: RE TRAI BAO VE (MUI TEN XANH)",
            "east_west_permissive": "DONG-TAY: XANH TRON / RE TRAI NHUONG DUONG",
            "east_west_yellow": "DONG-TAY: DEN VANG",
            "pedestrian_walk": "BO HANH: WALK - BAC NAM DI THANG XANH",
            "pedestrian_clearance": "BO HANH: DON DUONG - XE DUNG",
        }
        return labels[self.phase]


class FixedCycleBaseline:
    def __init__(self):
        self.phase_index = 0
        self.phase_elapsed = 0.0
        self.time = 0.0
        self.queues = {
            (direction, movement): deque()
            for direction in DIRECTIONS
            for movement in ("left", "through", "right")
        }
        self.service_elapsed = {lane: 0.0 for lane in self.queues}
        self.completed_vehicles = 0
        self.completed_wait_total = 0.0

    @property
    def phase(self):
        return TrafficLight.PHASES[self.phase_index]

    @property
    def average_wait(self):
        if not self.completed_vehicles:
            return None
        return self.completed_wait_total / self.completed_vehicles

    def register_vehicle(self, direction, movement):
        self.queues[(direction, movement)].append(self.time)

    def lane_is_green(self, direction, movement):
        phase = self.phase
        axis = "north_south" if direction in ("north", "south") else "east_west"
        if phase == "{}_protected_left".format(axis):
            return movement == "left"
        if phase == "{}_permissive".format(axis):
            if movement in ("through", "right"):
                return True
            opposing = OPPOSITE_APPROACH[direction]
            return not self.queues[(opposing, "through")]
        if movement == "right" and phase.endswith("_yellow"):
            return not phase.startswith(axis)
        return False

    def update(self, dt):
        self.time += dt
        self.phase_elapsed += dt
        while self.phase_elapsed >= FIXED_PHASE_DURATIONS[self.phase]:
            self.phase_elapsed -= FIXED_PHASE_DURATIONS[self.phase]
            self.phase_index = (self.phase_index + 1) % len(TrafficLight.PHASES)

        for lane, queue in self.queues.items():
            direction, movement = lane
            if not queue:
                self.service_elapsed[lane] = 0.0
                continue
            if not self.lane_is_green(direction, movement):
                self.service_elapsed[lane] = 0.0
                continue
            self.service_elapsed[lane] += dt
            headway = 2.0 if movement == "left" else 1.6
            while queue and self.service_elapsed[lane] >= headway:
                self.service_elapsed[lane] -= headway
                arrived_at = queue.popleft()
                self.completed_wait_total += self.time - arrived_at
                self.completed_vehicles += 1


class Car:
    def __init__(self, direction, movement, x, y):
        self.direction = direction
        self.movement = movement
        self.state = "approach"
        self.x = float(x)
        self.y = float(y)
        self.color = random.choice(CAR_COLORS)
        self.speed = random.uniform(125.0, 175.0)
        self.current_speed = 0.0
        self.stopped_seconds = 0.0
        self.travel_vector = APPROACH_VECTORS[direction]
        self.turn_elapsed = 0.0
        self.turn_duration = 1.35
        self.turn_curve = None
        self.turn_exit = None
        self.wait_seconds = 0.0

    @property
    def progress(self):
        return dot((self.x, self.y), self.travel_vector)

    def set_progress(self, progress):
        vx, vy = self.travel_vector
        current = self.progress
        distance = progress - current
        self.x += vx * distance
        self.y += vy * distance

    def start_turn(self, exit_name):
        start = (self.x, self.y)
        self.turn_curve = create_turn_curve(self.direction, exit_name, start)
        self.state = "turning"
        self.turn_elapsed = 0.0
        self.turn_exit = exit_name
        self.turn_duration = 1.35 if exit_name == LEFT_TURN_EXITS[self.direction] else 0.9

    def update_turn(self, dt):
        previous_position = (self.x, self.y)
        self.turn_elapsed = min(self.turn_elapsed + dt, self.turn_duration)
        t = self.turn_elapsed / self.turn_duration
        p0, p1, p2, p3 = self.turn_curve
        u = 1.0 - t
        self.x = u ** 3 * p0[0] + 3 * u ** 2 * t * p1[0] + 3 * u * t ** 2 * p2[0] + t ** 3 * p3[0]
        self.y = u ** 3 * p0[1] + 3 * u ** 2 * t * p1[1] + 3 * u * t ** 2 * p2[1] + t ** 3 * p3[1]
        if dt > 0:
            self.current_speed = math.hypot(
                self.x - previous_position[0],
                self.y - previous_position[1],
            ) / dt
        if t >= 1.0:
            self.travel_vector = APPROACH_VECTORS[OPPOSITE_APPROACH[self.turn_exit]]
            self.state = "departing"
            self.current_speed = max(
                self.current_speed,
                min(self.speed, 140.0),
            )

    def draw(self, surface):
        car_surface = car_sprite(self.color)
        vx, vy = self.travel_vector
        if self.state == "turning" and self.turn_curve:
            p0, p1, p2, p3 = self.turn_curve
            t = self.turn_elapsed / self.turn_duration
            u = 1.0 - t
            vx = 3 * u ** 2 * (p1[0] - p0[0]) + 6 * u * t * (p2[0] - p1[0]) + 3 * t ** 2 * (p3[0] - p2[0])
            vy = 3 * u ** 2 * (p1[1] - p0[1]) + 6 * u * t * (p2[1] - p1[1]) + 3 * t ** 2 * (p3[1] - p2[1])
        angle = round(-math.degrees(math.atan2(vy, vx)) / 5) * 5
        rotated = rotated_car_sprite(self.color, angle)
        surface.blit(rotated, rotated.get_rect(center=(round(self.x), round(self.y))))


class Pedestrian:
    def __init__(self, crosswalk, reverse=False):
        self.crosswalk = crosswalk
        self.reverse = reverse
        self.start = CROSSWALKS[crosswalk]["end" if reverse else "start"]
        self.end = CROSSWALKS[crosswalk]["start" if reverse else "end"]
        self.progress = 0.0
        self.speed = PEDESTRIAN_SPEED * random.uniform(0.9, 1.1)
        self.crossing = False
        self.waiting_position = self.start

    @property
    def position(self):
        if not self.crossing:
            return self.waiting_position
        return (
            self.start[0] + (self.end[0] - self.start[0]) * self.progress,
            self.start[1] + (self.end[1] - self.start[1]) * self.progress,
        )

    @property
    def complete(self):
        return self.progress >= 1.0

    def update(self, dt, walk_active):
        if not self.crossing and not walk_active:
            return
        self.crossing = True
        length = math.hypot(
            self.end[0] - self.start[0],
            self.end[1] - self.start[1],
        )
        self.progress = min(1.0, self.progress + self.speed * dt / length)

    def set_waiting_position(self, queue_index):
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        length = math.hypot(dx, dy)
        distance = 14.0 * (queue_index + 1)
        self.waiting_position = (
            self.start[0] - dx / length * distance,
            self.start[1] - dy / length * distance,
        )

    def draw(self, surface):
        x, y = self.position
        center = (round(x), round(y))
        pygame.draw.circle(surface, (245, 205, 165), (center[0], center[1] - 5), 4)
        pygame.draw.line(
            surface, (35, 40, 48), (center[0], center[1] - 1),
            (center[0], center[1] + 8), 3,
        )
        pygame.draw.line(
            surface, (35, 40, 48), (center[0], center[1] + 2),
            (center[0] - 5, center[1] + 7), 2,
        )
        pygame.draw.line(
            surface, (35, 40, 48), (center[0], center[1] + 2),
            (center[0] + 5, center[1] + 7), 2,
        )
        pygame.draw.line(
            surface, (35, 40, 48), (center[0], center[1] + 8),
            (center[0] - 4, center[1] + 13), 2,
        )
        pygame.draw.line(
            surface, (35, 40, 48), (center[0], center[1] + 8),
            (center[0] + 4, center[1] + 13), 2,
        )


class Simulation:
    def __init__(self):
        self.signal = TrafficLight()
        self.fixed_baseline = FixedCycleBaseline()
        self.cars = []
        self.pedestrians = []
        self.pedestrians_spawned = 0
        self.pedestrian_spawn_elapsed = 0.0
        self.pedestrian_spawn_index = 0
        self.queue_counts = {direction: 0 for direction in DIRECTIONS}
        self.spawn_timers = {direction: random.uniform(0.0, 1.0)
                             for direction in DIRECTIONS}
        self.spawn_intervals = {direction: random.uniform(1.3, 2.0)
                                for direction in DIRECTIONS}
        self.completed_left_turns = 0
        self.completed_right_turns = 0
        self.completed_vehicles = 0
        self.completed_wait_total = 0.0
        self.paused = False

    @property
    def walk_signal_active(self):
        return bool(self.signal.walk_crosswalks())

    @property
    def average_wait(self):
        if not self.completed_vehicles:
            return None
        return self.completed_wait_total / self.completed_vehicles

    @property
    def fixed_average_wait(self):
        return self.fixed_baseline.average_wait

    @property
    def ai_wait_reduction_percent(self):
        fixed_wait = self.fixed_average_wait
        ai_wait = self.average_wait
        if fixed_wait is None or ai_wait is None or fixed_wait <= 0:
            return None
        return (fixed_wait - ai_wait) / fixed_wait * 100

    def record_vehicle_passed(self, car):
        self.completed_vehicles += 1
        self.completed_wait_total += car.wait_seconds

    def spawn_car(self, direction):
        choice = random.random()
        movement = "left" if choice < 0.22 else (
            "right" if choice < 0.42 else "through"
        )
        lane_x, lane_y = LANES[direction][movement]
        if direction == "north":
            x, y = lane_x, -CAR_LENGTH
        elif direction == "south":
            x, y = lane_x, SCREEN_HEIGHT + CAR_LENGTH
        elif direction == "east":
            x, y = SCREEN_WIDTH + CAR_LENGTH, lane_y
        else:
            x, y = -CAR_LENGTH, lane_y

        vector = APPROACH_VECTORS[direction]
        start_progress = dot((x, y), vector)
        if any(car.direction == direction and car.movement == movement
               and car.state == "approach"
               and abs(car.progress - start_progress) < CAR_LENGTH + CAR_GAP
               for car in self.cars):
            return
        self.cars.append(Car(direction, movement, x, y))
        self.fixed_baseline.register_vehicle(direction, movement)

    def stop_progress(self, direction):
        return dot(STOP_POINTS[direction], APPROACH_VECTORS[direction])

    def is_inside_intersection(self, car):
        return (
            abs(car.x - CENTER_X) <= INTERSECTION_HALF_SIZE
            and abs(car.y - CENTER_Y) <= INTERSECTION_HALF_SIZE
        )

    def vehicles_cleared_for_walk(self):
        for car in self.cars:
            if self.is_inside_intersection(car):
                return False
            if car.state == "approach" and car.progress > self.stop_progress(car.direction):
                return False
        return True

    def update_pedestrians(self, dt):
        self.pedestrian_spawn_elapsed += dt
        while self.pedestrian_spawn_elapsed >= PEDESTRIAN_SPAWN_INTERVAL:
            self.pedestrian_spawn_elapsed -= PEDESTRIAN_SPAWN_INTERVAL
            crosswalk_index = self.pedestrian_spawn_index % len(DIRECTIONS)
            crosswalk = DIRECTIONS[crosswalk_index]
            reverse = (
                self.pedestrian_spawn_index // len(DIRECTIONS)
            ) % 2 == 1
            self.pedestrians.append(
                Pedestrian(crosswalk, reverse=reverse)
            )
            self.pedestrians_spawned += 1
            self.pedestrian_spawn_index += 1

        walk_crosswalks = set(self.signal.walk_crosswalks())
        waiting_queues = {}
        for pedestrian in self.pedestrians:
            walk_active = pedestrian.crosswalk in walk_crosswalks
            pedestrian.update(dt, walk_active)
            if not walk_active:
                queue_key = (pedestrian.crosswalk, pedestrian.reverse)
                queue_index = waiting_queues.get(queue_key, 0)
                pedestrian.set_waiting_position(queue_index)
                waiting_queues[queue_key] = queue_index + 1
        self.pedestrians = [
            pedestrian for pedestrian in self.pedestrians
            if not pedestrian.complete
        ]

    def pedestrian_conflict_active(self, car=None):
        if car is None:
            return any(pedestrian.crossing for pedestrian in self.pedestrians)

        conflict_crosswalks = {car.direction}
        if car.movement == "left":
            conflict_crosswalks.add(LEFT_TURN_EXITS[car.direction])
        elif car.movement == "right":
            conflict_crosswalks.add(RIGHT_TURN_EXITS[car.direction])
        return any(
            pedestrian.crossing
            and pedestrian.crosswalk in conflict_crosswalks
            for pedestrian in self.pedestrians
        )

    def permissive_left_is_clear(self, direction):
        opposing = OPPOSITE_APPROACH[direction]
        for car in self.cars:
            if car.direction != opposing:
                continue
            if car.state in ("turning", "departing") and self.is_inside_intersection(car):
                return False
            if car.movement == "through" and car.state == "approach":
                distance = self.stop_progress(opposing) - car.progress
                if (distance <= PERMISSIVE_CONFLICT_DISTANCE
                        and self.signal.through_color_for(opposing) == GREEN):
                    return False
            if self.is_inside_intersection(car):
                return False
        return True

    def right_turn_is_clear(self, direction):
        active_axis = "north_south" if direction in ("north", "south") else "east_west"
        for car in self.cars:
            if car.direction == direction:
                continue
            if self.is_inside_intersection(car):
                return False
            car_axis = "north_south" if car.direction in ("north", "south") else "east_west"
            if car_axis == active_axis:
                continue
            if car.state == "approach":
                distance = self.stop_progress(car.direction) - car.progress
                if (distance <= PERMISSIVE_CONFLICT_DISTANCE
                        and self.signal.through_color_for(car.direction) == GREEN):
                    return False
        return True

    def can_enter_intersection(self, car):
        stop_progress = self.stop_progress(car.direction)
        committed = car.progress > stop_progress
        if committed:
            return True
        if self.pedestrian_conflict_active(car):
            return False
        if car.movement == "through":
            return self.signal.through_color_for(car.direction) == GREEN
        if car.movement == "left":
            if self.signal.left_color_for(car.direction) == GREEN:
                if self.signal.pedestrians_walk:
                    return self.permissive_left_is_clear(car.direction)
                return True
            return (
                self.signal.through_color_for(car.direction) == GREEN
                and self.permissive_left_is_clear(car.direction)
            )
        if self.signal.pedestrians_walk or self.signal.pedestrian_clearance:
            return False
        if self.signal.through_color_for(car.direction) == GREEN:
            return True
        return (
            self.signal.through_color_for(car.direction) == RED
            and car.stopped_seconds >= STOP_DWELL_SECONDS
            and self.right_turn_is_clear(car.direction)
        )

    def turn_entry_is_clear(self, car):
        return not any(
            other is not car
            and (
                other.state == "turning"
                or self.is_inside_intersection(other)
            )
            for other in self.cars
        )

    def turn_exit_is_clear(self, car):
        exit_name = (
            LEFT_TURN_EXITS[car.direction]
            if car.movement == "left"
            else RIGHT_TURN_EXITS[car.direction]
        )
        curve = create_turn_curve(car.direction, exit_name, (car.x, car.y))
        end_x, end_y = curve[3]
        exit_vector = APPROACH_VECTORS[OPPOSITE_APPROACH[exit_name]]
        lateral_vector = (-exit_vector[1], exit_vector[0])

        for other in self.cars:
            if other is car:
                continue
            other_exit = car_outbound_vector(other)
            if other_exit != exit_vector:
                continue
            offset = (other.x - end_x, other.y - end_y)
            longitudinal_gap = dot(offset, exit_vector)
            lateral_gap = abs(dot(offset, lateral_vector))
            if (
                -MIN_CAR_CENTER_DISTANCE
                <= longitudinal_gap
                <= MIN_CAR_CENTER_DISTANCE * 1.5
                and lateral_gap <= CAR_WIDTH + CAR_GAP
            ):
                return False
        return True

    def count_waiting_queues(self):
        counts = {direction: 0 for direction in DIRECTIONS}
        for direction in DIRECTIONS:
            stop_progress = self.stop_progress(direction)
            for car in self.cars:
                if car.direction != direction or car.state != "approach":
                    continue
                distance_to_bar = stop_progress - car.progress
                if 0 <= distance_to_bar <= 240 and car.current_speed < 45:
                    counts[direction] += 1
        return counts

    def update_approach_lane(self, direction, movement, cars, dt):
        lane_cars = sorted(cars, key=lambda car: car.progress, reverse=True)
        leader_progress = None
        leader_speed = 0.0
        stop_progress = self.stop_progress(direction)
        for car in lane_cars:
            progress = car.progress
            turning_obstacles = [
                (other.progress, other.current_speed)
                for other in self.cars
                if other is not car
                and other.direction == direction
                and other.movement == movement
                and (
                    other.state == "turning"
                    or (
                        other.state == "departing"
                        and other.travel_vector == APPROACH_VECTORS[direction]
                    )
                )
                and progress < other.progress <= stop_progress + ROAD_WIDTH + CAR_LENGTH
            ]
            lane_leaders = turning_obstacles
            if leader_progress is not None:
                lane_leaders.append((leader_progress, leader_speed))
            nearest_leader = max(
                lane_leaders,
                key=lambda obstacle: obstacle[0],
                default=None,
            )
            safe_progress = (
                nearest_leader[0] - MIN_CAR_CENTER_DISTANCE
                if nearest_leader is not None
                else None
            )
            if safe_progress is not None and progress > safe_progress:
                car.set_progress(safe_progress)
                car.current_speed = 0.0
                progress = safe_progress

            distance_to_bar = max(0.0, stop_progress - progress)
            at_bar = distance_to_bar <= 0.75
            if at_bar and car.current_speed <= 2.0:
                car.stopped_seconds += dt
            elif not at_bar:
                car.stopped_seconds = 0.0

            can_go = self.can_enter_intersection(car)
            committed = progress > stop_progress
            if (movement in ("left", "right") and not committed
                    and not self.turn_entry_is_clear(car)):
                can_go = False
            if (movement == "left" and not committed
                    and not self.turn_exit_is_clear(car)):
                can_go = False
            if (movement in ("left", "right") and not committed
                    and any(
                        other is not car
                        and other.direction == direction
                        and other.movement == movement
                        and other.state == "turning"
                        for other in self.cars
                    )):
                can_go = False
            if (movement == "right" and not committed
                    and self.signal.through_color_for(direction) == RED):
                can_go = (
                    not self.signal.pedestrians_walk
                    and not self.signal.pedestrian_clearance
                    and not self.pedestrian_conflict_active(car)
                    and at_bar
                    and car.stopped_seconds >= STOP_DWELL_SECONDS
                    and self.right_turn_is_clear(direction)
                    and self.turn_entry_is_clear(car)
                )

            target_speed = car.speed if can_go else min(
                car.speed, math.sqrt(2 * DECELERATION * distance_to_bar)
            )
            if nearest_leader is not None:
                following_distance = (
                    nearest_leader[0] - progress - MIN_CAR_CENTER_DISTANCE
                )
                target_speed = min(target_speed, math.sqrt(
                    nearest_leader[1] ** 2
                    + 2 * DECELERATION * max(0.0, following_distance)
                ))
                target_speed = min(
                    target_speed,
                    nearest_leader[1]
                    + max(0.0, following_distance) / FOLLOWING_TIME_HEADWAY,
                )
            if car.current_speed < 8 and target_speed < 8:
                car.wait_seconds += dt
            speed_delta = DECELERATION * dt
            if target_speed < car.current_speed:
                car.current_speed = max(target_speed, car.current_speed - speed_delta)
            else:
                car.current_speed = min(target_speed, car.current_speed + speed_delta)
            next_progress = progress + car.current_speed * dt
            movement_limit = safe_progress
            if movement_limit is None and nearest_leader is not None:
                movement_limit = nearest_leader[0] - MIN_CAR_CENTER_DISTANCE
            if movement_limit is not None:
                next_progress = min(next_progress, movement_limit)
            if not can_go and not committed:
                movement_limit = (
                    stop_progress
                    if movement_limit is None
                    else min(movement_limit, stop_progress)
                )
                next_progress = min(next_progress, movement_limit)
            actual_speed = max(0.0, (next_progress - progress) / dt) if dt > 0 else 0.0
            car.set_progress(next_progress)
            if actual_speed < car.current_speed:
                car.current_speed = actual_speed
            if (movement in ("left", "right") and can_go
                    and car.progress >= stop_progress - 0.5):
                exit_name = (
                    LEFT_TURN_EXITS[direction]
                    if movement == "left" else RIGHT_TURN_EXITS[direction]
                )
                car.start_turn(exit_name)
                leader_progress = car.progress
                leader_speed = car.current_speed
                continue
            if movement == "through" and car.progress > stop_progress + ROAD_WIDTH + CAR_LENGTH:
                car.state = "departing"
                self.record_vehicle_passed(car)
            leader_progress = car.progress
            leader_speed = car.current_speed

    def update(self, dt):
        if self.paused:
            return

        self.fixed_baseline.update(dt)
        pedestrian_ready = self.vehicles_cleared_for_walk()
        self.signal.update(dt, self.queue_counts, pedestrian_ready)
        self.update_pedestrians(dt)

        for direction in DIRECTIONS:
            self.spawn_timers[direction] += dt
            if self.spawn_timers[direction] >= self.spawn_intervals[direction]:
                self.spawn_timers[direction] = 0.0
                self.spawn_intervals[direction] = random.uniform(1.3, 2.0)
                self.spawn_car(direction)

        previous_motion = {
            car: (car.x, car.y, car.turn_elapsed)
            for car in self.cars
        }
        previous_states = {
            car: (
                car.state,
                car.turn_curve,
                car.turn_exit,
                car.travel_vector,
            )
            for car in self.cars
        }
        for direction in DIRECTIONS:
            for movement in ("through", "left", "right"):
                lane_cars = [
                    car for car in self.cars
                    if car.direction == direction and car.movement == movement
                    and car.state == "approach"
                ]
                self.update_approach_lane(direction, movement, lane_cars, dt)

        for car in self.cars:
            if car.state == "turning":
                car.update_turn(dt)

        self.update_departing_vehicles(dt)
        self.resolve_pedestrian_collisions(previous_motion, previous_states)
        self.resolve_vehicle_collisions(previous_motion)
        for car in self.cars:
            if (
                previous_states[car][0] == "turning"
                and car.state == "departing"
            ):
                if car.movement == "left":
                    self.completed_left_turns += 1
                elif car.movement == "right":
                    self.completed_right_turns += 1
                self.record_vehicle_passed(car)

        self.cars = [
            car for car in self.cars
            if -CAR_LENGTH <= car.x <= SCREEN_WIDTH + CAR_LENGTH
            and -CAR_LENGTH <= car.y <= SCREEN_HEIGHT + CAR_LENGTH
        ]
        self.queue_counts = self.count_waiting_queues()

    def resolve_pedestrian_collisions(self, previous_motion, previous_states):
        pedestrians = [
            pedestrian for pedestrian in self.pedestrians
            if pedestrian.crossing
        ]
        if not pedestrians:
            return

        for car in self.cars:
            start_x, start_y, start_turn_elapsed = previous_motion[car]
            start_state, start_curve, start_exit, start_vector = (
                previous_states[car]
            )
            end_x, end_y = car.x, car.y
            end_state = car.state
            end_curve = car.turn_curve
            end_exit = car.turn_exit
            end_vector = car.travel_vector
            end_turn_elapsed = car.turn_elapsed
            clearance = (
                CAR_LENGTH / 2 + CAR_WIDTH / 2
                + PEDESTRIAN_COLLISION_BUFFER
            )
            nearby_pedestrians = [
                pedestrian for pedestrian in pedestrians
                if (
                    min(start_x, end_x) - clearance
                    <= pedestrian.position[0]
                    <= max(start_x, end_x) + clearance
                    and min(start_y, end_y) - clearance
                    <= pedestrian.position[1]
                    <= max(start_y, end_y) + clearance
                )
            ]
            if not nearby_pedestrians:
                continue
            distance = math.hypot(end_x - start_x, end_y - start_y)
            samples = max(1, math.ceil(distance / 5.0))
            collision = False
            for step in range(samples + 1):
                fraction = step / samples
                car.x = start_x + (end_x - start_x) * fraction
                car.y = start_y + (end_y - start_y) * fraction
                if start_state == "turning" or end_state == "turning":
                    car.state = "turning"
                    car.turn_curve = start_curve or end_curve
                    car.turn_exit = start_exit or end_exit
                    if start_state == "turning":
                        start_turn_elapsed_for_path = start_turn_elapsed
                    else:
                        start_turn_elapsed_for_path = 0.0
                    car.turn_elapsed = (
                        start_turn_elapsed_for_path
                        + (end_turn_elapsed - start_turn_elapsed_for_path)
                        * fraction
                    )
                for pedestrian in nearby_pedestrians:
                    if car_overlaps_pedestrian(car, pedestrian):
                        collision = True
                        break
                if collision:
                    break
            car.x, car.y = end_x, end_y
            car.x, car.y = end_x, end_y
            car.x, car.y = end_x, end_y
            car.turn_curve = end_curve
            car.turn_exit = end_exit
            car.travel_vector = end_vector
            car.turn_elapsed = end_turn_elapsed
            if collision:
                car.x, car.y, car.turn_elapsed = (
                    start_x,
                    start_y,
                    start_turn_elapsed,
                )
                car.state = start_state
                car.turn_curve = start_curve
                car.turn_exit = start_exit
                car.travel_vector = start_vector
                car.current_speed = 0.0

    def resolve_vehicle_collisions(self, previous_motion):
        for _ in range(len(self.cars)):
            found_collision = False
            for index, first in enumerate(self.cars):
                for second in self.cars[index + 1:]:
                    first_outbound = car_outbound_vector(first)
                    second_outbound = car_outbound_vector(second)
                    share_outbound_lane = (
                        first_outbound is not None
                        and first_outbound == second_outbound
                        and abs(
                            dot((first.x - second.x, first.y - second.y),
                                (-first_outbound[1], first_outbound[0]))
                        ) <= CAR_WIDTH + CAR_GAP / 2
                    )
                    share_approach_lane = (
                        first.direction == second.direction
                        and first.movement == second.movement
                        and first.state in ("approach", "turning", "departing")
                        and second.state in ("approach", "turning", "departing")
                        and all(
                            car.state != "departing"
                            or car.travel_vector == APPROACH_VECTORS[car.direction]
                            for car in (first, second)
                        )
                    )
                    same_lane = (
                        share_approach_lane or share_outbound_lane
                    )
                    if not car_collision_boxes_overlap(
                        first,
                        second,
                        include_safe_gap=same_lane,
                    ):
                        continue

                    first_start = previous_motion.get(
                        first, (first.x, first.y, first.turn_elapsed)
                    )
                    second_start = previous_motion.get(
                        second, (second.x, second.y, second.turn_elapsed)
                    )
                    same_route = (
                        first_outbound is not None
                        and first_outbound == second_outbound
                    ) or (
                        share_approach_lane
                        and first.travel_vector == second.travel_vector
                    ) or first.travel_vector == second.travel_vector
                    if same_route:
                        forward = (
                            first_outbound
                            if first_outbound == second_outbound
                            and first_outbound is not None
                            else first.travel_vector
                        )
                        first_progress = dot((first.x, first.y), forward)
                        second_progress = dot((second.x, second.y), forward)
                        trailing = first if first_progress < second_progress else second
                    elif first.state == "turning" and second.state != "turning":
                        trailing = second
                    elif second.state == "turning" and first.state != "turning":
                        trailing = first
                    elif first.state == "turning" and second.state == "turning":
                        trailing = (
                            first if first.turn_elapsed < second.turn_elapsed
                            else second
                        )
                    else:
                        first_motion = math.hypot(
                            first.x - first_start[0], first.y - first_start[1]
                        )
                        second_motion = math.hypot(
                            second.x - second_start[0], second.y - second_start[1]
                        )
                        trailing = first if first_motion >= second_motion else second

                    start = previous_motion.get(
                        trailing, (trailing.x, trailing.y, trailing.turn_elapsed)
                    )
                    trailing.x, trailing.y = start[0], start[1]
                    if trailing.state == "turning":
                        trailing.turn_elapsed = min(
                            trailing.turn_elapsed, start[2]
                        )
                    trailing.current_speed = 0.0
                    found_collision = True
            if not found_collision:
                return

    def update_departing_vehicles(self, dt):
        outbound_lanes = []
        for car in self.cars:
            if car.state != "departing":
                continue
            vx, vy = car.travel_vector
            lane_offset = car.y if vx else car.x
            lane = next(
                (
                    cars for lane_vector, offset, cars in outbound_lanes
                    if lane_vector == (vx, vy)
                    and abs(offset - lane_offset) <= CAR_WIDTH + CAR_GAP / 2
                ),
                None,
            )
            if lane is None:
                outbound_lanes.append(((vx, vy), lane_offset, [car]))
            else:
                lane.append(car)

        for (vx, vy), _, lane_cars in outbound_lanes:
            lane_cars.sort(
                key=lambda car: dot((car.x, car.y), (vx, vy)),
                reverse=True,
            )
            leader_progress = None
            leader_speed = 0.0
            for car in lane_cars:
                progress = dot((car.x, car.y), (vx, vy))
                target_speed = min(car.speed * 1.15, 195.0)
                if leader_progress is not None:
                    gap = leader_progress - progress - MIN_CAR_CENTER_DISTANCE
                    target_speed = min(
                        target_speed,
                        math.sqrt(
                            leader_speed ** 2
                            + 2 * DECELERATION * max(0.0, gap)
                        ),
                    )
                    target_speed = min(
                        target_speed,
                        leader_speed
                        + max(0.0, gap) / FOLLOWING_TIME_HEADWAY,
                    )

                speed_delta = ACCELERATION * dt
                car.current_speed = min(
                    target_speed,
                    car.current_speed + speed_delta,
                )
                next_progress = progress + car.current_speed * dt
                if leader_progress is not None:
                    next_progress = min(
                        next_progress,
                        leader_progress - MIN_CAR_CENTER_DISTANCE,
                    )
                actual_speed = (
                    max(0.0, (next_progress - progress) / dt)
                    if dt > 0 else 0.0
                )
                car.x += vx * (next_progress - progress)
                car.y += vy * (next_progress - progress)
                car.current_speed = min(car.current_speed, actual_speed)
                leader_progress = next_progress
                leader_speed = car.current_speed


def run_adaptive_smoke_checks():
    signal = TrafficLight()
    north_south_backlog = {
        "north": 8,
        "south": 4,
        "east": 1,
        "west": 0,
    }
    signal.update(4.0, north_south_backlog)
    if signal.phase != "north_south_protected_left":
        raise RuntimeError("AI khong keo dai pha xanh khi hang cho lon hon.")
    signal.update(4.1, north_south_backlog)
    if signal.phase != "north_south_permissive":
        raise RuntimeError("Pha xanh vuot qua gioi han toi da.")

    competing_backlog = {
        "north": 0,
        "south": 0,
        "east": 5,
        "west": 4,
    }
    signal.update(10.0, competing_backlog)
    if signal.phase != "north_south_yellow":
        raise RuntimeError("AI khong chuyen pha khi huong doi dien un tac hon.")

    simulation = Simulation()
    stop_x, stop_y = STOP_POINTS["north"]
    lane_x = LANES["north"]["through"][0]
    simulation.cars = [
        Car("north", "through", lane_x, stop_y - 10),
        Car("north", "through", lane_x, stop_y - 10 - CAR_LENGTH - CAR_GAP),
    ]
    measured = simulation.count_waiting_queues()
    if measured["north"] != 2:
        raise RuntimeError("Dem hang cho theo huong khong chinh xac.")

    permissive = Simulation()
    permissive.signal.phase_index = TrafficLight.PHASES.index("north_south_permissive")
    if (permissive.signal.left_color_for("north") != RED
            or permissive.signal.through_color_for("north") != GREEN):
        raise RuntimeError("Pha permissive khong hien thi xanh tron dung.")
    opposing_lane = LANES["south"]["through"][0]
    opposing_stop = STOP_POINTS["south"][1]
    opposing_car = Car("south", "through", opposing_lane, opposing_stop + 80)
    opposing_car.current_speed = 45
    permissive.cars = [opposing_car]
    left_car = Car("north", "left", LANES["north"]["left"][0], STOP_POINTS["north"][1])
    permissive.cars.append(left_car)
    if permissive.can_enter_intersection(left_car):
        raise RuntimeError("Xe re trai permissive khong nhuong xe doi dien.")
    permissive.signal.phase_index = TrafficLight.PHASES.index(
        "north_south_protected_left"
    )
    if (permissive.signal.left_color_for("north") != GREEN
            or permissive.signal.through_color_for("north") != RED):
        raise RuntimeError("Pha re trai bao ve khong tach rieng voi pha permissive.")
    permissive.cars.clear()
    if not permissive.permissive_left_is_clear("north"):
        raise RuntimeError("Pha permissive khong cho re khi duong doi dien trong.")

    anti_gridlock = Simulation()
    anti_gridlock.signal.phase_index = TrafficLight.PHASES.index(
        "north_south_protected_left"
    )
    left_car = Car(
        "north", "left", LANES["north"]["left"][0],
        STOP_POINTS["north"][1],
    )
    exit_curve = create_turn_curve(
        "north", LEFT_TURN_EXITS["north"], (left_car.x, left_car.y)
    )
    exit_vector = APPROACH_VECTORS[
        OPPOSITE_APPROACH[LEFT_TURN_EXITS["north"]]
    ]
    exit_blocker = Car(
        "east",
        "through",
        exit_curve[3][0] - exit_vector[0] * 20,
        exit_curve[3][1] - exit_vector[1] * 20,
    )
    exit_blocker.state = "departing"
    exit_blocker.travel_vector = exit_vector
    anti_gridlock.cars = [left_car, exit_blocker]
    anti_gridlock.update_approach_lane("north", "left", [left_car], 1 / 60)
    if left_car.state != "approach" or left_car.progress > anti_gridlock.stop_progress("north"):
        raise RuntimeError("Xe re trai di vao giao lo khi loi ra dang bi chan.")

    anti_gridlock.cars = [left_car]
    anti_gridlock.update_approach_lane("north", "left", [left_car], 1 / 60)
    if left_car.state != "turning":
        raise RuntimeError("Xe re trai khong duoc vao khi loi ra thong thoang.")
    anti_gridlock.signal.phase_index = TrafficLight.PHASES.index(
        "east_west_permissive"
    )
    anti_gridlock.signal.elapsed = 0.0
    turning_speeds = []
    for _ in range(90):
        left_car.update_turn(1 / 60)
        turning_speeds.append(left_car.current_speed)
    if left_car.state != "departing":
        raise RuntimeError("Xe re trai da vao giao lo khong thoat khi doi pha.")
    if max(turning_speeds) <= 80:
        raise RuntimeError("Toc do xe khong duoc cap nhat khi dang re.")
    if left_car.current_speed < min(left_car.speed, 140.0):
        raise RuntimeError("Xe re trai roi giao lo voi toc do qua cham.")

    rtor = Simulation()
    right_lane = LANES["north"]["right"][0]
    right_stop = STOP_POINTS["north"][1]
    right_car = Car("north", "right", right_lane, right_stop)
    cross_traffic = Car(
        "east", "through", CENTER_X, CENTER_Y,
    )
    rtor.cars = [right_car, cross_traffic]
    right_car.stopped_seconds = STOP_DWELL_SECONDS + 0.1
    if rtor.can_enter_intersection(right_car):
        raise RuntimeError("RTOR khong nhuong xe dang o trong giao lo.")
    rtor.cars = [right_car]
    right_car.stopped_seconds = 0.0
    if rtor.can_enter_intersection(right_car):
        raise RuntimeError("RTOR duoc cho phep khi chua dung du thoi gian.")
    rtor.update_approach_lane("north", "right", [right_car], STOP_DWELL_SECONDS + 0.1)
    if right_car.state != "turning":
        raise RuntimeError("RTOR khong duoc tiep tuc sau khi dung va quan sat.")

    stopping = Simulation()
    stop_line = stopping.stop_progress("north")
    near_bar = Car("north", "through", lane_x, stop_y - 14)
    near_bar.current_speed = 70
    trailing = Car(
        "north", "through", lane_x,
        stop_y - 14 - CAR_LENGTH - CAR_GAP,
    )
    trailing.current_speed = 70
    stopping.cars = [near_bar, trailing]
    for _ in range(20):
        stopping.update_approach_lane(
            "north", "through", [near_bar, trailing], 0.1
        )
        if (near_bar.progress > stop_line
                or trailing.progress > near_bar.progress - MIN_CAR_CENTER_DISTANCE):
            raise RuntimeError(
                "Xe vuot stop bar hoac khong giu khoang cach an toan."
            )

    queue_simulation = Simulation()
    queue_simulation.signal.phase_index = TrafficLight.PHASES.index(
        "pedestrian_clearance"
    )
    queue_stop = queue_simulation.stop_progress("north")
    queued_cars = [
        Car(
            "north",
            "through",
            lane_x,
            queue_stop - MIN_CAR_CENTER_DISTANCE * index,
        )
        for index in range(1, 7)
    ]
    for car in queued_cars:
        car.current_speed = 170.0
    queue_simulation.cars = queued_cars
    for _ in range(180):
        queue_simulation.update_approach_lane(
            "north", "through", queued_cars, 1.0 / 30.0
        )
        ordered_queue = sorted(
            queued_cars, key=lambda car: car.progress, reverse=True
        )
        if ordered_queue[0].progress > queue_stop:
            raise RuntimeError("Hang xe vuot qua stop bar khi den do.")
        if any(
            front.progress - following.progress
            < MIN_CAR_CENTER_DISTANCE - 1e-6
            for front, following in zip(ordered_queue, ordered_queue[1:])
        ):
            raise RuntimeError("Khoang cach an toan trong hang xe bi vi pham.")

    left_lane_x = LANES["north"]["left"][0]
    turning_leader = Car(
        "north", "left", left_lane_x, stop_line,
    )
    turning_leader.start_turn("east")
    following_car = Car(
        "north", "left", left_lane_x,
        stop_line - CAR_LENGTH - CAR_GAP + 4,
    )
    following_car.current_speed = 45
    stopping.cars = [turning_leader, following_car]
    stopping.update_approach_lane(
        "north", "left", [following_car], 0.1
    )
    if following_car.progress > turning_leader.progress - MIN_CAR_CENTER_DISTANCE:
        raise RuntimeError("Xe phia sau vuot qua xe dang bat dau re.")
    if following_car.state != "approach":
        raise RuntimeError("Xe thu hai bat dau re khi xe truoc chua ra khoi duong re.")

    clearing = Simulation()
    clearing.signal.phase_index = TrafficLight.PHASES.index("pedestrian_walk")
    clearing.pedestrians = [Pedestrian("north")]
    committed_car = Car(
        "north", "through", lane_x, clearing.stop_progress("north") + 12,
    )
    committed_car.current_speed = 45.0
    clearing.cars = [committed_car]
    prior_progress = committed_car.progress
    clearing.update_approach_lane("north", "through", [committed_car], 0.1)
    if committed_car.progress <= prior_progress or committed_car.current_speed <= 0:
        raise RuntimeError("Xe da vao giao lo bi dung khi dang thoat giao lo.")

    committed_right_turn = Car(
        "north",
        "right",
        right_lane,
        clearing.stop_progress("north") + 12,
    )
    committed_right_turn.current_speed = 45.0
    clearing.pedestrians.clear()
    clearing.cars = [committed_right_turn]
    clearing.update_approach_lane(
        "north", "right", [committed_right_turn], 0.1
    )
    if committed_right_turn.state != "turning":
        raise RuntimeError("Xe re phai da qua stop line khong tiep tuc thoat.")

    turning_exit = Car(
        "north", "left", left_lane_x, clearing.stop_progress("north") + 12,
    )
    turning_exit.current_speed = 0.0
    turning_exit.start_turn("east")
    turning_exit.update_turn(turning_exit.turn_duration)
    if turning_exit.state != "departing":
        raise RuntimeError("Xe dung lai sau khi hoan tat re qua giao lo.")
    exit_position = (turning_exit.x, turning_exit.y)
    clearing.cars = [turning_exit]
    clearing.update_departing_vehicles(0.1)
    if (turning_exit.x, turning_exit.y) == exit_position:
        raise RuntimeError("Xe khong tiep tuc chay sau khi thoat giao lo.")
    if turning_exit.current_speed <= 0:
        raise RuntimeError("Xe khong tang toc sau khi hoan tat re.")

    through_exit = Car(
        "north",
        "through",
        lane_x,
        clearing.stop_progress("north") + ROAD_WIDTH + CAR_LENGTH + 1,
    )
    through_exit.current_speed = 0.0
    clearing.signal.phase_index = TrafficLight.PHASES.index("pedestrian_walk")
    clearing.cars = [through_exit]
    clearing.update_approach_lane("north", "through", [through_exit], 0.1)
    if through_exit.state != "departing" or through_exit.current_speed <= 0:
        raise RuntimeError("Xe di thang dung lai sau khi qua giao lo.")

    departure_test = Simulation()
    departure_leader = Car("north", "through", 400, 300)
    departure_leader.state = "departing"
    departure_leader.travel_vector = (1, 0)
    departure_leader.current_speed = 55.0
    departure_follower = Car("north", "through", 340, 300)
    departure_follower.state = "departing"
    departure_follower.travel_vector = (1, 0)
    departure_follower.current_speed = 0.0
    departure_test.cars = [departure_leader, departure_follower]
    for _ in range(120):
        departure_test.update_departing_vehicles(1.0 / 60.0)
        if (departure_leader.x - departure_follower.x
                < MIN_CAR_CENTER_DISTANCE - 1e-6):
            raise RuntimeError("Xe dang roi giao lo bi ap sat xe dan dau.")
    if departure_follower.current_speed <= 0:
        raise RuntimeError("Xe phia sau khong tang toc sau khi roi giao lo.")
    if departure_follower.x >= departure_leader.x:
        raise RuntimeError("Xe phia sau vuot qua xe dan dau khi roi giao lo.")

    slow_leader = Car("north", "through", 500, 300)
    slow_leader.state = "departing"
    slow_leader.travel_vector = (1, 0)
    slow_leader.speed = 8.0
    slow_leader.current_speed = 8.0
    fast_follower = Car("north", "through", 260, 300)
    fast_follower.state = "departing"
    fast_follower.travel_vector = (1, 0)
    fast_follower.speed = 170.0
    fast_follower.current_speed = 170.0
    departure_test.cars = [slow_leader, fast_follower]
    for _ in range(300):
        departure_test.update_departing_vehicles(1.0 / 60.0)
        if (slow_leader.x - fast_follower.x
                < MIN_CAR_CENTER_DISTANCE - 1e-6):
            raise RuntimeError("Xe phia sau ap sat xe dan dau chay cham.")
    if fast_follower.current_speed > slow_leader.current_speed + 10.0:
        raise RuntimeError("Xe phia sau khong giam toc theo xe dan dau cham.")

    collision_test = Simulation()
    collision_leader = Car("north", "through", 400, 300)
    collision_follower = Car("north", "through", 340, 300)
    collision_test.cars = [collision_leader, collision_follower]
    prior_motion = {
        collision_leader: (collision_leader.x, collision_leader.y, 0.0),
        collision_follower: (collision_follower.x, collision_follower.y, 0.0),
    }
    collision_follower.x = collision_leader.x - 35
    collision_test.resolve_vehicle_collisions(prior_motion)
    if car_collision_boxes_overlap(collision_leader, collision_follower):
        raise RuntimeError("Phat hien va cham khong loai bo duoc xe chong len nhau.")

    for direction in DIRECTIONS:
        vx, vy = APPROACH_VECTORS[direction]
        stop_point = STOP_POINTS[direction]
        bar_front = dot(
            (stop_point[0] + vx * CAR_LENGTH / 2,
             stop_point[1] + vy * CAR_LENGTH / 2),
            (vx, vy),
        )
        crosswalk = CROSSWALKS[direction]
        center = (
            (crosswalk["start"][0] + crosswalk["end"][0]) / 2,
            (crosswalk["start"][1] + crosswalk["end"][1]) / 2,
        )
        crosswalk_near_edge = dot(center, (vx, vy)) - CROSSWALK_DEPTH / 2
        if crosswalk_near_edge - bar_front < CROSSWALK_GAP - 0.1:
            raise RuntimeError("Crosswalk khong nam sau vach dung xe.")
        if abs(center[0] - CENTER_X) < ROAD_WIDTH / 2 and abs(center[1] - CENTER_Y) < ROAD_WIDTH / 2:
            raise RuntimeError("Crosswalk bi dat vao giua giao lo.")
        stripes = crosswalk_stripe_segments(direction)
        if not stripes:
            raise RuntimeError("Thieu vach son crosswalk cho mot huong.")
        for start, end in stripes:
            if direction in ("north", "south"):
                if start[1] != end[1] or abs(end[0] - start[0]) != ROAD_WIDTH:
                    raise RuntimeError("Crosswalk khong ke ngang qua nhanh Bac-Nam.")
            elif start[0] != end[0] or abs(end[1] - start[1]) != ROAD_WIDTH:
                raise RuntimeError("Crosswalk khong ke ngang qua nhanh Dong-Tay.")

    pedestrian_signal = TrafficLight()
    pedestrian_signal.phase_index = TrafficLight.PHASES.index("east_west_yellow")
    pedestrian_signal.elapsed = TrafficLight.YELLOW_DURATION - 0.01
    pedestrian_signal.update(0.02, {direction: 0 for direction in DIRECTIONS})
    if not pedestrian_signal.pedestrians_walk:
        raise RuntimeError("Pha WALK khong bat sau pha vang.")
    if pedestrian_signal.pedestrian_walk_axis != "east_west":
        raise RuntimeError("Huong WALK khong luan phien sau chu ky giao thong.")
    if any(
        pedestrian_signal.through_color_for(direction)
        != (GREEN if direction in ("east", "west") else RED)
        for direction in DIRECTIONS
    ):
        raise RuntimeError("Pha WALK khong dong bo xanh voi huong xe tuong thich.")
    if pedestrian_signal.walk_crosswalks() != ("north", "south"):
        raise RuntimeError("WALK khong chuyen sang cap crosswalk tiep theo.")
    if any(
        pedestrian_signal.left_color_for(direction)
        != (
            GREEN
            if (direction in ("north", "south"))
            == (pedestrian_signal.pedestrian_walk_axis == "north_south")
            else RED
        )
        for direction in DIRECTIONS
    ):
        raise RuntimeError("Mui ten re khong xanh cung truc trong pha WALK.")
    first_walk_axis = pedestrian_signal.pedestrian_walk_axis
    empty_queues = {direction: 0 for direction in DIRECTIONS}
    for _ in range(500):
        pedestrian_signal.update(0.1, empty_queues)
        if (
            pedestrian_signal.pedestrians_walk
            and pedestrian_signal.pedestrian_walk_axis != first_walk_axis
        ):
            break
    else:
        raise RuntimeError("Den WALK khong luan phien sang huong doi dien.")
    if pedestrian_signal.walk_crosswalks() != ("east", "west"):
        raise RuntimeError("Chu ky WALK tiep theo khong doi cap crosswalk.")

    synchronized_signal = TrafficLight()
    synchronized_signal.phase_index = TrafficLight.PHASES.index(
        "north_south_permissive"
    )
    if synchronized_signal.walk_crosswalks() != ("east", "west"):
        raise RuntimeError("Den xanh Bac-Nam chua cap WALK cho crosswalk Dong-Tay.")
    if any(
        synchronized_signal.through_color_for(direction) == GREEN
        and direction not in ("north", "south")
        for direction in DIRECTIONS
    ):
        raise RuntimeError("WALK duoc cap cho huong xe khong cung truc.")
    synchronized_signal.phase_index = TrafficLight.PHASES.index(
        "east_west_permissive"
    )
    if synchronized_signal.walk_crosswalks() != ("north", "south"):
        raise RuntimeError("Den xanh Dong-Tay chua cap WALK cho crosswalk Bac-Nam.")
    synchronized_simulation = Simulation()
    synchronized_simulation.signal.phase_index = TrafficLight.PHASES.index(
        "east_west_permissive"
    )
    synchronized_simulation.pedestrian_spawn_elapsed = (
        PEDESTRIAN_SPAWN_INTERVAL
    )
    synchronized_simulation.update_pedestrians(0.01)
    if (
        not synchronized_simulation.walk_signal_active
        or not synchronized_simulation.pedestrians
        or synchronized_simulation.pedestrians[0].crosswalk not in DIRECTIONS
    ):
        raise RuntimeError("Khong tao duoc hang nguoi di bo.")
    if synchronized_simulation.pedestrians[0].crossing != (
        synchronized_simulation.pedestrians[0].crosswalk
        in synchronized_simulation.signal.walk_crosswalks()
    ):
        raise RuntimeError("Nguoi di bo khong cho dung den WALK.")
    synchronized_signal.phase_index = TrafficLight.PHASES.index(
        "north_south_protected_left"
    )
    if not synchronized_signal.walk_crosswalks():
        raise RuntimeError("Khong co WALK trong pha re trai bao ve.")

    for phase in TrafficLight.PHASES:
        synchronized_signal.phase_index = TrafficLight.PHASES.index(phase)
        if not synchronized_signal.walk_crosswalks():
            raise RuntimeError(
                "Ca hai huong crosswalk deu WAIT trong pha {}.".format(phase)
            )

    pedestrian_queues = Simulation()
    pedestrian_queues.signal.phase_index = TrafficLight.PHASES.index(
        "north_south_permissive"
    )
    waiting_pedestrian = Pedestrian("north")
    walking_pedestrian = Pedestrian("east")
    pedestrian_queues.pedestrians = [
        waiting_pedestrian,
        walking_pedestrian,
    ]
    pedestrian_queues.update_pedestrians(0.1)
    if waiting_pedestrian.crossing or not walking_pedestrian.crossing:
        raise RuntimeError("Nguoi di bo khong cho dung den WALK.")
    waiting_position = waiting_pedestrian.position
    pedestrian_queues.signal.phase_index = TrafficLight.PHASES.index(
        "east_west_permissive"
    )
    pedestrian_queues.update_pedestrians(0.1)
    if not waiting_pedestrian.crossing:
        raise RuntimeError("Nguoi di bo khong qua duong khi den WALK bat.")
    if waiting_pedestrian.position == waiting_position:
        raise RuntimeError("Nguoi di bo khong di xuong duong khi co WALK.")

    busy_pedestrian_simulation = Simulation()
    busy_pedestrian_simulation.signal.phase_index = TrafficLight.PHASES.index(
        "north_south_permissive"
    )
    busy_pedestrian_simulation.pedestrian_spawn_elapsed = (
        PEDESTRIAN_SPAWN_INTERVAL - 0.01
    )
    busy_pedestrian_simulation.update(0.02)
    if not busy_pedestrian_simulation.pedestrians:
        raise RuntimeError("Khong co nguoi di bo cho trong pha den xanh.")
    busy_pedestrian_simulation.update_pedestrians(
        PEDESTRIAN_SPAWN_INTERVAL * (len(DIRECTIONS) - 1)
    )
    if {
        pedestrian.crosswalk
        for pedestrian in busy_pedestrian_simulation.pedestrians
    } != set(DIRECTIONS):
        raise RuntimeError("Nguoi di bo khong xep hang o ca bon crosswalk.")
    active_crosswalks = set(
        busy_pedestrian_simulation.signal.walk_crosswalks()
    )
    if any(
        pedestrian.crossing != (pedestrian.crosswalk in active_crosswalks)
        for pedestrian in busy_pedestrian_simulation.pedestrians
    ):
        raise RuntimeError("Nguoi di bo khong chap hanh tin hieu crosswalk.")

    pedestrian_simulation = Simulation()
    pedestrian_simulation.signal.phase_index = TrafficLight.PHASES.index(
        "pedestrian_walk"
    )
    if not pedestrian_simulation.walk_signal_active:
        raise RuntimeError("Walk khong bat theo pha pedestrian_walk.")
    pedestrian_simulation.cars = [
        Car(
            "north",
            "through",
            LANES["north"]["through"][0],
            CENTER_Y,
        )
    ]
    if not pedestrian_simulation.walk_signal_active:
        raise RuntimeError("Xe xanh song song khong duoc lam tat tin hieu Walk.")
    pedestrian_simulation.pedestrian_spawn_elapsed = PEDESTRIAN_SPAWN_INTERVAL
    pedestrian_simulation.update_pedestrians(0.01)
    if not pedestrian_simulation.pedestrians or pedestrian_simulation.pedestrians_spawned != 1:
        raise RuntimeError("Khong tao duoc nguoi di bo trong pha WALK.")
    pedestrian = pedestrian_simulation.pedestrians[0]
    if pedestrian.crossing != (
        pedestrian.crosswalk in pedestrian_simulation.signal.walk_crosswalks()
    ):
        raise RuntimeError("Nguoi di bo khong cho dung den WALK.")
    crosswalk = CROSSWALKS[pedestrian.crosswalk]
    px, py = pedestrian.position
    if crosswalk["orientation"] == "horizontal":
        if abs(py - crosswalk["start"][1]) > 0.01:
            raise RuntimeError("Nguoi di bo khong di tren crosswalk ngang.")
    elif abs(px - crosswalk["start"][0]) > 0.01:
        raise RuntimeError("Nguoi di bo khong di tren crosswalk doc.")

    pedestrian_collision_simulation = Simulation()
    collision_crosswalk = CROSSWALKS["north"]
    collision_pedestrian = Pedestrian("north")
    collision_pedestrian.crossing = True
    collision_pedestrian.progress = (
        LANES["north"]["through"][0] - collision_crosswalk["start"][0]
    ) / (
        collision_crosswalk["end"][0] - collision_crosswalk["start"][0]
    )
    collision_car = Car(
        "north", "through", LANES["north"]["through"][0], 100
    )
    pedestrian_collision_simulation.cars = [collision_car]
    pedestrian_collision_simulation.pedestrians = [collision_pedestrian]
    previous_motion = {
        collision_car: (
            collision_car.x,
            collision_car.y,
            collision_car.turn_elapsed,
        )
    }
    previous_states = {
        collision_car: (
            collision_car.state,
            collision_car.turn_curve,
            collision_car.turn_exit,
            collision_car.travel_vector,
        )
    }
    collision_car.y = collision_pedestrian.position[1] - 23
    pedestrian_collision_simulation.resolve_pedestrian_collisions(
        previous_motion, previous_states
    )
    if collision_car.y != previous_motion[collision_car][1]:
        raise RuntimeError("Xe di xuyen qua nguoi di bo tren crosswalk.")
    if collision_car.current_speed != 0:
        raise RuntimeError("Xe khong dung lai truoc nguoi di bo.")

    crosswalk_drive = Simulation()
    crosswalk_drive.signal.phase_index = TrafficLight.PHASES.index(
        "north_south_permissive"
    )
    crosswalk_drive.pedestrians = [Pedestrian("east")]
    crosswalk_drive.pedestrians[0].crossing = True
    north_crosswalk_y = (
        CROSSWALKS["north"]["start"][1]
        + CROSSWALKS["north"]["end"][1]
    ) / 2
    crosswalk_car = Car(
        "north", "through", LANES["north"]["through"][0], north_crosswalk_y,
    )
    crosswalk_car.current_speed = crosswalk_car.speed
    crosswalk_drive.cars = [crosswalk_car]
    crossing_speed = crosswalk_car.current_speed
    if not crosswalk_drive.can_enter_intersection(crosswalk_car):
        raise RuntimeError("Xe bi dung boi nguoi di bo o crosswalk khong xung dot.")
    crosswalk_drive.update_approach_lane(
        "north", "through", [crosswalk_car], 0.1
    )
    if crosswalk_car.current_speed < crossing_speed - 1e-6:
        raise RuntimeError("Xe giam toc khi qua crosswalk dang trong.")

    crosswalk_drive.signal.phase_index = TrafficLight.PHASES.index(
        "east_west_permissive"
    )
    crosswalk_drive.pedestrians = [Pedestrian("north")]
    crosswalk_drive.pedestrians[0].crossing = True
    crosswalk_yield_car = Car(
        "north", "through", LANES["north"]["through"][0],
        STOP_POINTS["north"][1],
    )
    if crosswalk_drive.can_enter_intersection(crosswalk_yield_car):
        raise RuntimeError("Xe khong nhuong nguoi di bo tren crosswalk cung huong.")

    turning_car = Car(
        "north", "right", LANES["north"]["right"][0],
        STOP_POINTS["north"][1],
    )
    turning_car.stopped_seconds = STOP_DWELL_SECONDS + 0.1
    pedestrian_simulation.pedestrians = [Pedestrian("north")]
    pedestrian_simulation.pedestrians[0].crossing = True
    pedestrian_simulation.cars = [turning_car]
    if pedestrian_simulation.can_enter_intersection(turning_car):
        raise RuntimeError("Xe duoc re khi nguoi di bo dang qua duong.")
    committed_car = Car(
        "north", "through", LANES["north"]["through"][0],
        STOP_POINTS["north"][1],
    )
    committed_car.set_progress(
        pedestrian_simulation.stop_progress("north") + 1.0
    )
    if not pedestrian_simulation.can_enter_intersection(committed_car):
        raise RuntimeError("Xe da vao giao lo bi dung giua giao lo.")
    pedestrian_simulation.signal.phase_index = TrafficLight.PHASES.index(
        "north_south_protected_left"
    )
    if pedestrian_simulation.can_enter_intersection(turning_car):
        raise RuntimeError("Xe re trai khong nhuong nguoi di bo tren crosswalk.")
    left_turning_car = Car(
        "north", "left", LANES["north"]["left"][0],
        STOP_POINTS["north"][1],
    )
    if pedestrian_simulation.can_enter_intersection(left_turning_car):
        raise RuntimeError("Xe re trai khong uu tien nguoi di bo.")
    through_car = Car(
        "north", "through", LANES["north"]["through"][0],
        STOP_POINTS["north"][1],
    )
    if pedestrian_simulation.can_enter_intersection(through_car):
        raise RuntimeError("Xe di thang khong nhuong nguoi di bo.")
    pedestrian_simulation.signal.phase_index = TrafficLight.PHASES.index(
        "pedestrian_clearance"
    )
    if pedestrian_simulation.can_enter_intersection(turning_car):
        raise RuntimeError("RTOR duoc cho phep trong pha don crosswalk.")
    pedestrian_count = pedestrian_simulation.pedestrians_spawned
    pedestrian_simulation.update_pedestrians(
        PEDESTRIAN_SPAWN_INTERVAL + 0.01
    )
    if pedestrian_simulation.pedestrians_spawned <= pedestrian_count:
        raise RuntimeError("Khong co them nguoi di bo cho tai giao lo ban ron.")

    pedestrian_simulation.signal.phase_index = TrafficLight.PHASES.index(
        "pedestrian_clearance"
    )
    pedestrian_simulation.signal.elapsed = PEDESTRIAN_CLEARANCE_DURATION - 0.01
    pedestrian_simulation.signal.update(
        0.02, {direction: 0 for direction in DIRECTIONS}
    )
    if pedestrian_simulation.signal.phase != "north_south_protected_left":
        raise RuntimeError("Pha don crosswalk khong ket thuc dung thoi gian.")

    fixed_baseline = FixedCycleBaseline()
    fixed_baseline.register_vehicle("north", "left")
    fixed_baseline.update(2.1)
    if fixed_baseline.completed_vehicles != 1 or fixed_baseline.average_wait is None:
        raise RuntimeError("Khong tinh duoc thoi gian cho chu ky co dinh.")

    analytics = Simulation()
    analytics.completed_vehicles = 2
    analytics.completed_wait_total = 6.0
    analytics.fixed_baseline.completed_vehicles = 2
    analytics.fixed_baseline.completed_wait_total = 10.0
    if analytics.ai_wait_reduction_percent != 40.0:
        raise RuntimeError("So sanh hieu suat AI voi chu ky co dinh bi sai.")


def draw_arrow(surface, center, vector, color, size=15, width=3):
    cx, cy = center
    vx, vy = vector
    tail = (round(cx - vx * size * 0.55), round(cy - vy * size * 0.55))
    tip = (round(cx + vx * size * 0.55), round(cy + vy * size * 0.55))
    pygame.draw.line(surface, color, tail, tip, width)
    side = (-vy, vx)
    for sign in (-1, 1):
        point = (round(tip[0] - vx * size * 0.45 + side[0] * size * 0.35 * sign),
                 round(tip[1] - vy * size * 0.45 + side[1] * size * 0.35 * sign))
        pygame.draw.line(surface, color, tip, point, width)


def bezier_point(curve, t):
    p0, p1, p2, p3 = curve
    u = 1.0 - t
    return (
        u ** 3 * p0[0] + 3 * u ** 2 * t * p1[0] + 3 * u * t ** 2 * p2[0] + t ** 3 * p3[0],
        u ** 3 * p0[1] + 3 * u ** 2 * t * p1[1] + 3 * u * t ** 2 * p2[1] + t ** 3 * p3[1],
    )


def crosswalk_stripe_segments(crosswalk_name):
    crosswalk = CROSSWALKS[crosswalk_name]
    center_x = (crosswalk["start"][0] + crosswalk["end"][0]) / 2
    center_y = (crosswalk["start"][1] + crosswalk["end"][1]) / 2
    segments = []
    if crosswalk["orientation"] == "horizontal":
        first = round(center_y - CROSSWALK_DEPTH / 2)
        last = round(center_y + CROSSWALK_DEPTH / 2)
        for y in range(first, last, 10):
            segments.append(((CENTER_X - ROAD_WIDTH / 2, y),
                             (CENTER_X + ROAD_WIDTH / 2, y)))
    else:
        first = round(center_x - CROSSWALK_DEPTH / 2)
        last = round(center_x + CROSSWALK_DEPTH / 2)
        for x in range(first, last, 10):
            segments.append(((x, CENTER_Y - ROAD_WIDTH / 2),
                             (x, CENTER_Y + ROAD_WIDTH / 2)))
    return segments


def draw_left_turn_guides(surface):
    left = CENTER_X - ROAD_WIDTH // 2
    right = CENTER_X + ROAD_WIDTH // 2
    top = CENTER_Y - ROAD_WIDTH // 2
    bottom = CENTER_Y + ROAD_WIDTH // 2
    surface.set_clip(pygame.Rect(left + 3, top + 3, ROAD_WIDTH - 6, ROAD_WIDTH - 6))
    for direction in DIRECTIONS:
        lane_x, lane_y = LANES[direction]["left"]
        stop_x, stop_y = STOP_POINTS[direction]
        start = (
            float(lane_x if lane_x is not None else stop_x),
            float(lane_y if lane_y is not None else stop_y),
        )
        curve = create_turn_curve(direction, LEFT_TURN_EXITS[direction], start)
        previous = bezier_point(curve, 0.0)
        for segment in range(1, 49):
            current = bezier_point(curve, segment / 48)
            if segment % 4 in (1, 2):
                pygame.draw.line(
                    surface,
                    (215, 215, 195),
                    (round(previous[0]), round(previous[1])),
                    (round(current[0]), round(current[1])),
                    3,
                )
            previous = current
    surface.set_clip(None)


def draw_road(surface):
    surface.fill(BACKGROUND)
    left = CENTER_X - ROAD_WIDTH // 2
    right = CENTER_X + ROAD_WIDTH // 2
    top = CENTER_Y - ROAD_WIDTH // 2
    bottom = CENTER_Y + ROAD_WIDTH // 2
    pygame.draw.rect(surface, ROAD, (left, 0, ROAD_WIDTH, SCREEN_HEIGHT))
    pygame.draw.rect(surface, ROAD, (0, top, SCREEN_WIDTH, ROAD_WIDTH))

    for x in (left, right):
        pygame.draw.line(surface, ROAD_EDGE, (x, 0), (x, top), 3)
        pygame.draw.line(surface, ROAD_EDGE, (x, bottom), (x, SCREEN_HEIGHT), 3)
    for y in (top, bottom):
        pygame.draw.line(surface, ROAD_EDGE, (0, y), (left, y), 3)
        pygame.draw.line(surface, ROAD_EDGE, (right, y), (SCREEN_WIDTH, y), 3)

    north_south_segments = ((0, top), (bottom, SCREEN_HEIGHT))
    for lane_offset in (-76, -40, 40, 76):
        x = CENTER_X + lane_offset
        for start_y, end_y in north_south_segments:
            for y in range(start_y, end_y, 36):
                pygame.draw.line(surface, LANE_MARK, (x, y), (x, y + 21), 3)

    east_west_segments = ((0, left), (right, SCREEN_WIDTH))
    for lane_offset in (-76, -40, 40, 76):
        y = CENTER_Y + lane_offset
        for start_x, end_x in east_west_segments:
            for x in range(start_x, end_x, 36):
                pygame.draw.line(surface, LANE_MARK, (x, y), (x + 21, y), 3)
    pygame.draw.line(surface, WHITE, (CENTER_X, 0), (CENTER_X, top), 2)
    pygame.draw.line(surface, WHITE, (CENTER_X, bottom), (CENTER_X, SCREEN_HEIGHT), 2)
    pygame.draw.line(surface, WHITE, (0, CENTER_Y), (left, CENTER_Y), 2)
    pygame.draw.line(surface, WHITE, (right, CENTER_Y), (SCREEN_WIDTH, CENTER_Y), 2)
    draw_left_turn_guides(surface)

    for crosswalk_name in DIRECTIONS:
        for start, end in crosswalk_stripe_segments(crosswalk_name):
            pygame.draw.line(
                surface,
                WHITE,
                (round(start[0]), round(start[1])),
                (round(end[0]), round(end[1])),
                6,
            )

    for direction in DIRECTIONS:
        vx, vy = APPROACH_VECTORS[direction]
        stop_x, stop_y = STOP_POINTS[direction]
        for movement in ("right", "through", "left"):
            lane_x, lane_y = LANES[direction][movement]
            center = (lane_x if lane_x is not None else stop_x,
                      lane_y if lane_y is not None else stop_y)
            center = (center[0] - vx * 72, center[1] - vy * 72)
            if movement == "through":
                arrow = (vx, vy)
            elif movement == "left":
                arrow = (vy, -vx)
            else:
                arrow = (-vy, vx)
            for distance in (72, 145):
                marking_center = (
                    center[0] - vx * (distance - 72),
                    center[1] - vy * (distance - 72),
                )
                draw_arrow(
                    surface,
                    marking_center,
                    arrow,
                    WHITE if movement == "left" else LANE_MARK,
                    size=18 if movement == "left" else 16,
                    width=3,
                )

    for direction in DIRECTIONS:
        vx, vy = APPROACH_VECTORS[direction]
        stop_x, stop_y = STOP_POINTS[direction]
        bar_x = stop_x + vx * CAR_LENGTH / 2
        bar_y = stop_y + vy * CAR_LENGTH / 2
        if direction in ("north", "south"):
            pygame.draw.line(
                surface, WHITE,
                (left, round(bar_y)), (right, round(bar_y)), 5,
            )
        else:
            pygame.draw.line(
                surface, WHITE,
                (round(bar_x), top), (round(bar_x), bottom), 5,
            )


SIGNAL_POSITIONS = {
    "north": ((CENTER_X - 58, CENTER_Y - 145),
              (CENTER_X - 22, CENTER_Y - 145),
              (CENTER_X - 94, CENTER_Y - 145)),
    "south": ((CENTER_X + 58, CENTER_Y + 145),
              (CENTER_X + 22, CENTER_Y + 145),
              (CENTER_X + 94, CENTER_Y + 145)),
    "east": ((CENTER_X + 145, CENTER_Y - 58),
             (CENTER_X + 145, CENTER_Y - 22),
             (CENTER_X + 145, CENTER_Y - 94)),
    "west": ((CENTER_X - 145, CENTER_Y + 58),
             (CENTER_X - 145, CENTER_Y + 22),
             (CENTER_X - 145, CENTER_Y + 94)),
}


def draw_signal(surface, direction, signal):
    through_center, left_center, right_center = SIGNAL_POSITIONS[direction]
    left_vector = (APPROACH_VECTORS[direction][1],
                   -APPROACH_VECTORS[direction][0])
    for center, color, arrow_vector in (
        (through_center, signal.through_color_for(direction), None),
        (left_center, signal.left_color_for(direction), left_vector),
        (right_center, signal.through_color_for(direction), None),
    ):
        rect = pygame.Rect(0, 0, 28, 28)
        rect.center = center
        pygame.draw.rect(surface, (25, 27, 29), rect, border_radius=8)
        if arrow_vector:
            draw_arrow(surface, center, arrow_vector, color, size=15, width=3)
        else:
            pygame.draw.circle(surface, color, center, 8)


def draw_pedestrian_signals(surface, signal, font):
    positions = {
        "north": (CENTER_X + ROAD_WIDTH / 2 + 31,
                  CROSSWALKS["north"]["start"][1]),
        "south": (CENTER_X - ROAD_WIDTH / 2 - 31,
                  CROSSWALKS["south"]["start"][1]),
        "east": (CROSSWALKS["east"]["start"][0],
                 CENTER_Y + ROAD_WIDTH / 2 + 30),
        "west": (CROSSWALKS["west"]["start"][0],
                 CENTER_Y - ROAD_WIDTH / 2 - 30),
    }
    for direction, (x, y) in positions.items():
        walk_active = direction in signal.walk_crosswalks()
        label = "WALK" if walk_active else "WAIT"
        color = GREEN if walk_active else RED
        rect = pygame.Rect(0, 0, 48, 22)
        rect.center = (round(x), round(y))
        pygame.draw.rect(surface, (25, 27, 29), rect, border_radius=4)
        text = font.render(label, True, color)
        surface.blit(text, text.get_rect(center=rect.center))


def draw_analytics_dashboard(surface, simulation, font):
    panel_width = 218
    panel_height = 116
    panel = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
    panel.fill((18, 22, 27, 220))
    panel_x, panel_y = SCREEN_WIDTH - panel_width - 14, 12
    surface.blit(panel, (panel_x, panel_y))

    ai_wait = simulation.average_wait
    fixed_wait = simulation.fixed_average_wait
    reduction = simulation.ai_wait_reduction_percent
    lines = [
        ("ANALYTICS", (120, 205, 255)),
        ("Vehicles passed: {}".format(simulation.completed_vehicles), TEXT),
        (
            "Avg wait AI: {}".format(
                "--" if ai_wait is None else "{:.1f}s".format(ai_wait)
            ),
            TEXT,
        ),
        (
            "Fixed cycle: {}".format(
                "--" if fixed_wait is None else "{:.1f}s".format(fixed_wait)
            ),
            TEXT,
        ),
    ]
    if reduction is None:
        comparison = "AI vs fixed: collecting data"
        comparison_color = YELLOW
    elif reduction >= 0:
        comparison = "AI improvement: +{:.1f}%".format(reduction)
        comparison_color = GREEN
    else:
        comparison = "AI vs fixed: {:.1f}%".format(reduction)
        comparison_color = YELLOW
    lines.append((comparison, comparison_color))

    for index, (text, color) in enumerate(lines):
        rendered = font.render(text, True, color)
        surface.blit(rendered, (panel_x + 10, panel_y + 7 + index * 21))


def main():
    parser = argparse.ArgumentParser(description="Mo phong giao lo 4 huong.")
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="Chay kiem tra ngan bang cua so SDL ao roi tu thoat.",
    )
    args = parser.parse_args()
    if args.smoke_test:
        random.seed(7)
        run_adaptive_smoke_checks()
        random.seed(7)

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("SmartFlow - U.S. Adaptive Intersection")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 25)
    small_font = pygame.font.Font(None, 20)
    dashboard_font = pygame.font.Font(None, 20)
    simulation = Simulation()
    frames = 0
    smoke_frames = 4800
    phases_at_start = set(simulation.signal.visited_phases)

    try:
        running = True
        while running and (not args.smoke_test or frames < smoke_frames):
            dt = 1 / 60 if args.smoke_test else min(clock.tick(60) / 1000, 0.05)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                    elif event.key == pygame.K_SPACE:
                        simulation.paused = not simulation.paused

            simulation.update(dt)
            if args.smoke_test and any(
                car_collision_boxes_overlap(
                    first, second, include_safe_gap=False
                )
                for index, first in enumerate(simulation.cars)
                for second in simulation.cars[index + 1:]
            ):
                raise RuntimeError(
                    "Smoke test phat hien hai xe chong len nhau."
                )
            draw_road(screen)
            for car in simulation.cars:
                car.draw(screen)
            for pedestrian in simulation.pedestrians:
                pedestrian.draw(screen)
            for direction in DIRECTIONS:
                draw_signal(screen, direction, simulation.signal)
            draw_pedestrian_signals(
                screen, simulation.signal, small_font
            )
            draw_analytics_dashboard(screen, simulation, dashboard_font)

            status_text = simulation.signal.phase_label()
            if (simulation.signal.pedestrians_walk
                    and not simulation.walk_signal_active):
                status_text = "BO HANH: WAIT - CHO XE TRONG GIAO LOI ROI HET"
            if simulation.signal.green_extension > 0:
                status_text += "  AI +{:.1f}s".format(
                    simulation.signal.green_extension
                )
            status = font.render(status_text, True, TEXT)
            screen.blit(status, (18, 14))
            controls = small_font.render(
                "SPACE: tam dung/tiep tuc   ESC/Q: thoat   Xe:{}  Trai:{}  Phai:{}  Bo hanh:{}"
                .format(
                    len(simulation.cars),
                    simulation.completed_left_turns,
                    simulation.completed_right_turns,
                    len(simulation.pedestrians),
                ),
                True,
                TEXT,
            )
            screen.blit(controls, (18, 40))
            queue_status = small_font.render(
                "AI HANG CHO  Bac:{}  Nam:{}  Dong:{}  Tay:{}".format(
                    simulation.queue_counts["north"],
                    simulation.queue_counts["south"],
                    simulation.queue_counts["east"],
                    simulation.queue_counts["west"],
                ),
                True,
                YELLOW,
            )
            screen.blit(queue_status, (18, 64))
            rules_status = small_font.render(
                "WALK: xe dung tat ca huong  |  Xe re phai/re trai nhuong nguoi qua duong",
                True,
                TEXT,
            )
            screen.blit(rules_status, (18, 88))
            if simulation.paused:
                paused = font.render("DA TAM DUNG", True, YELLOW)
                screen.blit(paused, paused.get_rect(center=(CENTER_X, CENTER_Y)))

            pygame.display.flip()
            frames += 1

        if args.smoke_test:
            expected_phases = set(TrafficLight.PHASES)
            visited_phases = simulation.signal.visited_phases | phases_at_start
            if simulation.completed_left_turns == 0:
                raise RuntimeError("Smoke test khong hoan tat duoc xe re trai.")
            if simulation.completed_right_turns == 0:
                raise RuntimeError("Smoke test khong hoan tat duoc xe re phai.")
            if simulation.completed_vehicles == 0 or simulation.average_wait is None:
                raise RuntimeError("Analytics khong ghi nhan xe qua giao lo.")
            if simulation.fixed_average_wait is None:
                raise RuntimeError("Baseline chu ky co dinh chua hoan tat xe nao.")
            if simulation.pedestrians_spawned == 0:
                raise RuntimeError("Smoke test khong tao nguoi di bo trong pha WALK.")
            if not expected_phases.issubset(visited_phases):
                raise RuntimeError("Smoke test khong di qua du cac pha den.")
            print(
                "Smoke test thanh cong: {} khung hinh, {} xe qua giao lo, "
                "cho AI {:.1f}s / chu ky co dinh {:.1f}s, {} xe re trai, "
                "{} xe re phai, {} pha den.".format(
                    frames,
                    simulation.completed_vehicles,
                    simulation.average_wait,
                    simulation.fixed_average_wait,
                    simulation.completed_left_turns,
                    simulation.completed_right_turns,
                    len(visited_phases),
                )
            )
    finally:
        pygame.quit()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
