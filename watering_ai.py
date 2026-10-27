"""Offline inference for the explicitly simulated watering exercise (stdlib only)."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FEATURES = ('soil_moisture_pct', 'temperature_c', 'humidity_pct', 'hours_since_water')
LIMITS = ((0, 100), (5, 45), (10, 100), (0, 72))
CENTERS = (50.0, 25.0, 50.0, 24.0)
SCALES = (50.0, 20.0, 50.0, 24.0)


def validate_observation(observation):
    values = []
    for name, (low, high) in zip(FEATURES, LIMITS):
        value = observation.get(name)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not low <= value <= high):
            raise ValueError(f'{name} must be a finite number in [{low}, {high}]')
        values.append(float(value))
    return values


def normalized(values):
    return [(v - c) / s for v, c, s in zip(values, CENTERS, SCALES)]


def probability(weights, bias, values):
    score = bias + sum(w * x for w, x in zip(weights, normalized(values)))
    return 1.0 / (1.0 + math.exp(-max(-60.0, min(60.0, score))))


class WateringModel:
    def __init__(self, path=ROOT / 'ai_model' / 'watering_model.json'):
        content = Path(path).read_bytes()
        model = json.loads(content)
        if (model.get('schema_version') != 1
                or model.get('algorithm') != 'logistic_regression'
                or model.get('training_data_source') != 'synthetic'
                or model.get('features') != list(FEATURES)
                or model.get('centers') != list(CENTERS)
                or model.get('scales') != list(SCALES)):
            raise ValueError('Unsupported or inconsistent model metadata')
        self.weights, self.bias = model['weights'], model['bias']
        if (len(self.weights) != len(FEATURES)
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                           and math.isfinite(v) for v in [*self.weights, self.bias])):
            raise ValueError('Invalid model coefficients')
        self.sha256 = hashlib.sha256(content).hexdigest()

    def predict(self, observation):
        p = probability(self.weights, self.bias, validate_observation(observation))
        # This is a model score, not a calibrated probability of plant health.
        action = 'water' if p >= 0.60 else 'skip' if p <= 0.40 else 'review'
        return dict(action=action, model_water_probability=round(p, 6),
                    model_sha256=self.sha256)


class SimulatedInspection:
    def __init__(self, path=ROOT / 'simulated_plants.json'):
        self.model = WateringModel()
        content = Path(path).read_bytes()
        config = json.loads(content)
        if config.get('data_source') != 'simulated' or config.get('schema_version') != 1:
            raise ValueError('This exercise only accepts explicitly simulated observations')
        rows = config['plants']
        if len(rows) != 4 or [row.get('waypoint') for row in rows] != [1, 2, 3, 4]:
            raise ValueError('Exactly four plants in waypoint order 1, 2, 3, 4 are required')
        for row in rows:
            if not isinstance(row.get('plant_id'), str) or not row['plant_id']:
                raise ValueError('Each simulated plant needs a plant_id')
            validate_observation(row)
        self.plants = rows
        self.observations_sha256 = hashlib.sha256(content).hexdigest()

    def inspect(self, waypoint):
        row = self.plants[waypoint - 1]
        inputs = {name: row[name] for name in FEATURES}
        result = self.model.predict(inputs)
        label = {'water': '水やり必要', 'skip': '水やり不要', 'review': '判定保留・散水見送り'}[result['action']]
        return dict(data_source='simulated', actuator='log_only', plant_id=row['plant_id'],
                    inputs=inputs, **result,
                    message=f'【模擬AI】地点{waypoint} {row["plant_id"]}: {label}')
