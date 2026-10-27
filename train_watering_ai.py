"""Reproducibly learn a small demo model from SYNTHETIC labels; no API/packages."""
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import random

from watering_ai import FEATURES, LIMITS, CENTERS, SCALES, normalized, probability

ROOT = Path(__file__).resolve().parent


def generate(seed=20261026, count=1200):
    rng = random.Random(seed)
    rows = []
    for _ in range(count):
        values = [round(rng.uniform(lo, hi), 3) for lo, hi in LIMITS]
        moisture, temperature, humidity, hours = values
        # An invented teaching rule, NOT an agronomic standard or a field label.
        threshold = 35 + .35*(temperature-25) - .10*(humidity-50) + .10*(hours-24)
        label = int(moisture < threshold + rng.gauss(0, 1.5))
        rows.append((values, label))
    rng.shuffle(rows)
    return rows


def evaluate(rows, weights, bias):
    confusion = {'true_water': 0, 'true_skip': 0, 'false_water': 0, 'missed_water': 0}
    covered = correct_covered = correct = 0
    for values, target in rows:
        p = probability(weights, bias, values)
        predicted = int(p >= .5)
        correct += predicted == target
        key = ('true_water' if target else 'false_water') if predicted else ('missed_water' if target else 'true_skip')
        confusion[key] += 1
        if p >= .6 or p <= .4:
            covered += 1
            correct_covered += predicted == target
    return dict(samples=len(rows), accuracy_at_0_5=correct/len(rows),
                confident_coverage=covered/len(rows), review_count=len(rows)-covered,
                accuracy_when_not_review=correct_covered/covered if covered else None,
                confusion_at_0_5=confusion)


def main():
    rows = generate()
    train, validation, test = rows[:800], rows[800:1000], rows[1000:]
    data = io.StringIO(newline='')
    writer = csv.writer(data, lineterminator='\n')
    writer.writerow(['split', *FEATURES, 'synthetic_water_label'])
    for split, group in [('train', train), ('validation', validation), ('test', test)]:
        for values, label in group:
            writer.writerow([split, *values, label])
    encoded = data.getvalue().encode('utf-8')
    weights, bias = [0.0]*4, 0.0
    samples = [(normalized(v), y) for v, y in train]
    # Full-batch cross-entropy gradient descent with light L2 regularization.
    for _ in range(2200):
        gradient, bias_gradient = [0.0]*4, 0.0
        for x, target in samples:
            z = bias + sum(w*v for w, v in zip(weights, x))
            p = 1 / (1 + math.exp(-max(-60., min(60., z))))
            error = p-target
            bias_gradient += error
            for j in range(4):
                gradient[j] += error*x[j]
        weights = [w - .25*(g/len(train) + .001*w) for w, g in zip(weights, gradient)]
        bias -= .25*bias_gradient/len(train)
    metadata = dict(schema_version=1, algorithm='logistic_regression',
                    training_data_source='synthetic', seed=20261026,
                    features=list(FEATURES), centers=list(CENTERS), scales=list(SCALES),
                    weights=weights, bias=bias, training_samples=len(train),
                    dataset_sha256=hashlib.sha256(encoded).hexdigest(),
                    teaching_rule='moisture < 35 + .35*(temp-25) - .10*(humidity-50) + .10*(hours-24) + Normal(0,1.5)',
                    limitation='Synthetic labels only; no claim of real plant watering accuracy.')
    report = dict(validation=evaluate(validation, weights, bias),
                  test=evaluate(test, weights, bias), training_samples=len(train),
                  split='800 train / 200 validation / 200 held-out test; seed fixed before training',
                  limitation=metadata['limitation'])
    directory = ROOT / 'ai_model'
    directory.mkdir(exist_ok=True)
    (directory / 'synthetic_watering.csv').write_bytes(encoded)
    (directory / 'watering_model.json').write_text(json.dumps(metadata, indent=2)+'\n', encoding='utf-8')
    (directory / 'evaluation.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
