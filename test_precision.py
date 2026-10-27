import copy
import json
import math
from pathlib import Path
import unittest
from dose_model import DoseModel
from precision_plants import PlantWorld

ROOT=Path(__file__).resolve().parent

class PrecisionTests(unittest.TestCase):
    def setUp(self):
        self.rows=json.loads((ROOT/'simulated_plants.json').read_text())['plants']
        self.model=DoseModel()

    def test_leaf_condition_changes_dose_at_same_soil_moisture(self):
        row=dict(self.rows[0],soil_moisture_pct=35,leaf_wilt_pct=5)
        low=self.model.predict(row)['target_ml']
        high=self.model.predict(dict(row,leaf_wilt_pct=90))['target_ml']
        self.assertGreater(high,low+25)

    def test_already_wet_soil_does_not_receive_more_water(self):
        self.assertEqual(self.model.predict(dict(self.rows[0],soil_moisture_pct=80,leaf_wilt_pct=95))['target_ml'],0)

    def test_invalid_leaf_observation_is_rejected(self):
        for v in (math.nan,math.inf,-1,101,None):
            with self.subTest(v=v),self.assertRaises(ValueError):
                self.model.predict(dict(self.rows[0],leaf_wilt_pct=v))

    def test_delivery_is_idempotent_and_requires_monotonic_counter(self):
        world=PlantWorld(self.rows)
        before=copy.deepcopy(world.rows[0])
        row,_=world.apply('plant_A','session',20.)
        row,_=world.apply('plant_A','session',20.)
        self.assertAlmostEqual(row['soil_moisture_pct'],before['soil_moisture_pct']+5)
        self.assertAlmostEqual(row['leaf_wilt_pct'],before['leaf_wilt_pct']-10)
        with self.assertRaises(ValueError): world.apply('plant_A','session',19.)

    def test_after_feedback_next_observation_reduces_water_to_zero(self):
        world=PlantWorld(self.rows)
        first=self.model.predict(world.rows[0])
        row,_=world.apply('plant_A','one',first['target_ml'])
        second=self.model.predict(row)
        self.assertGreater(first['target_ml'],100)
        self.assertEqual(second['target_ml'],0)

if __name__=='__main__': unittest.main()
