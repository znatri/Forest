import pyaudio
import sys
import numpy as np
import aubio
import time
import rtmidi
from numpy import interp



silence_flag = False
silence_count = 0
def extract_energy(audio_block, thresholdindB):
    global silence_count, silence_flag
    rms = np.sqrt(np.mean(np.square(audio_block)))
    dB = 10 * np.log10(rms)
    if dB < thresholdindB:
        silence_count += 1
        if silence_count > (blocksPerBeat * 4):
            silence_flag = True
        else:
            silence_flag = False
    else:
        silence_count = 0
        silence_flag = False
        
    return dB, silence_flag


def send_midi(channel, note, velocity):
    midiout = rtmidi.MidiOut()
    available_ports = midiout.get_ports()
    if available_ports:
        midiout.open_port(0)
    else:
        midiout.open_virtual_port("My virtual output")
    with midiout:
        note_on = [channel, note, velocity]
        midiout.send_message(note_on)
    del midiout


def scale_values(source_low, source_high, dest_low, dest_high, data):
    # m = interp1d([source_low, source_high], [dest_low, dest_high])
    m = interp(data, [source_low, source_high], [dest_low, dest_high])
    return int(m)


# initialise pyaudio
p = pyaudio.PyAudio()
buffer_size = 1024
# open stream

pyaudio_format = pyaudio.paFloat32
n_channels = 1
samplerate = 44100
stream = p.open(format=pyaudio_format,
                channels=n_channels,
                rate=samplerate,
                input=True,
                frames_per_buffer=buffer_size)


# setup pitch
tolerance = 0.8
threshold = 0.2
win_s = 2048  # fft size
hop_s = buffer_size  # hop size

onset_o = aubio.onset("mkl", win_s, hop_s, samplerate)
onset_o.set_silence(-60.0)
onset_o.set_threshold(threshold)

tempo_o = aubio.tempo("default", win_s, hop_s, samplerate)
tempo_o.set_threshold(threshold)

print("*** starting recording")

blockedEnergies = []
TEMPO = 80
tempo_time = [0]
current_tempo = []
blocksPerBeat = (60/TEMPO) * (samplerate/hop_s)
onset_count = 0
energyTrack = 0
# scale = [48, 50, 52, 53, 55, 57, 59, 60]
count = 0
while True:

    try:
        audiobuffer = stream.read(buffer_size)
        signal = np.frombuffer(audiobuffer, dtype=np.float32)    

        onset = onset_o(signal)
        tempo = tempo_o(signal)

        if onset:
            print("onset")
            onset_count = 1
            # send_midi(0x92, 60, 127) # Channel 3

        # Get energy
        energyIndB, silence_flag = extract_energy(signal, -28.0)
        blockedEnergies.append(energyIndB)
        if len(blockedEnergies) == int(blocksPerBeat):
            energyAverage = np.sum(blockedEnergies) / blocksPerBeat
            energyTrack = energyAverage
            blockedEnergies = []

        if onset_count == 1 and not silence_flag:
            if tempo:
                # tempo_time.append(time.time())
                # TEMPO = int(60/abs(tempo_time[-1] - tempo_time[-2]))
                send_midi(0x91, 80, scale_values(-23., -17., 10, 40, energyTrack)) # Channel 2
                print(energyTrack)
                count += 1

    except KeyboardInterrupt:
        print("*** Ctrl+C pressed, exiting")
        break

print("*** done recording")
stream.stop_stream()
stream.close()
p.terminate()


