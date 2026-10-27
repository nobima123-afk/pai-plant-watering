import json
import math
from pathlib import Path
import tempfile
import unittest

from watering_ai import SimulatedInspection, WateringModel


class WateringTests(unittest.TestCase):
    def test_dry_and_wet_plants_produce_both_actions(self):
        demo = SimulatedInspection()
        self.assertEqual([demo.inspect(i)['action'] for i in range(1, 5)],
                         ['water', 'skip', 'water', 'skip'])
        self.assertTrue(all(demo.inspect(i)['data_source'] == 'simulated' for i in range(1, 5)))

    def test_invalid_inputs_never_produce_a_watering_decision(self):
        demo = SimulatedInspection()
        observation = {k: demo.plants[0][k] for k in ('soil_moisture_pct', 'temperature_c', 'humidity_pct', 'hours_since_water')}
        for value in (math.nan, math.inf, -1, 101, None, '18', True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                demo.model.predict(dict(observation, soil_moisture_pct=value))

    def test_moisture_changes_decision_without_changing_environment(self):
        model = WateringModel()
        common = dict(temperature_c=25, humidity_pct=50, hours_since_water=24)
        self.assertEqual(model.predict(dict(common, soil_moisture_pct=10))['action'], 'water')
        self.assertEqual(model.predict(dict(common, soil_moisture_pct=90))['action'], 'skip')
        # At the learned decision boundary the controller defers to review.
        low, high = 0., 100.
        for _ in range(40):
            mid = (low+high)/2
            p = model.predict(dict(common, soil_moisture_pct=mid))['model_water_probability']
            if p > .5:
                low = mid
            else:
                high = mid
        self.assertEqual(model.predict(dict(common, soil_moisture_pct=mid))['action'], 'review')

    def test_real_sensor_data_cannot_be_mislabeled_as_demo(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'plants.json'
            path.write_text(json.dumps({'schema_version': 1, 'data_source': 'real', 'plants': []}))
            with self.assertRaises(ValueError):
                SimulatedInspection(path)


if __name__ == '__main__':
    unittest.main()
