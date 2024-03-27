from random import random
import pysine
from time import sleep

TEMPO = 90  # bpm
seconds = (1 / 90) * 60  # seconds per beat


def build_cdf(model):
    cdf = [[[] for x in model]] * len(model)
    for i in range(len(model)):
        cumsum = 0
        for connection in range(len(model[i])):
            cumsum += model[i][connection]
            cdf[i][connection] = cumsum
    return cdf


notes_hz = [
    ("C4", 262),
    ("D4", 294),
    ("E4", 330),
    ("G4", 392),
    ("A4", 440),
    ("B4", 494),
]
notes_model = [[1 / len(notes_hz) for note in notes_hz]] * len(notes_hz)
notes_model_cdf = build_cdf(notes_model)

note_lengths = [4, 2, 1, 0.5, 0.25]
length_model = [[0.05, 0.15, 0.55, 0.15, 0.1]] * len(note_lengths)
length_model_cdf = build_cdf(length_model)

octaves = [0.5, 1, 2]
octave_model = [[0.9, 0.05, 0.05], [0.05, 0.9, 0.05], [0.05, 0.05, 0.9]]
octave_model_cdf = build_cdf(octave_model)

note_choice = int(random() * len(notes_hz))
length_choice = int(random() * len(note_lengths))
octave_choice = int(random() * len(octaves))
for i in range(20):
    print(f"\nnote: {i}")
    played = False
    note_roll = random() * 1
    for note_option in range(len(notes_model_cdf[note_choice])):
        if (
            not played and note_roll <= notes_model_cdf[note_choice][note_option]
        ):  # got note
            length_roll = random() * 1
            for length_option in range(len(length_model_cdf[length_choice])):
                if (
                    not played
                    and length_roll <= length_model_cdf[length_choice][length_option]
                ):  # got length
                    octave_roll = random() * 1
                    for octave_option in range(len(octave_model_cdf[octave_choice])):
                        if (
                            not played
                            and octave_roll
                            <= octave_model_cdf[octave_choice][octave_option]
                        ):  # got octave
                            note_choice = note_option
                            length_choice = length_option
                            octave_choice = octave_option
                            print(f"note_roll: {note_roll}")
                            print(f"length_roll: {length_roll}")
                            print(f"octave_roll: {octave_roll}")
                            print(notes_hz[note_choice][0])
                            print(f'length: {note_lengths[length_choice]}')
                            print(f'octave: {octaves[octave_choice]}')
                            pysine.sine(
                                frequency=notes_hz[note_option][1]
                                * octaves[octave_choice],
                                duration=seconds * (note_lengths[length_choice]),
                            )
                            played = True
                            break