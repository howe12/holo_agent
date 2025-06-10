#!/usr/bin/env python3
from modelscope.outputs import OutputKeys
from modelscope.pipelines import pipeline
from modelscope.utils.constant import Tasks

text = '拼接法和参数法是两种Text-To-Speech(TTS)技术路线。近年来参数TTS系统获得了广泛的应用，故此处仅涉及参数法。'
model_id = '/home/leo/.cache/modelscope/hub/iic/speech_sambert-hifigan_tts_zh-cn_16k'
sambert_hifigan_tts = pipeline(task=Tasks.text_to_speech, model=model_id)
output = sambert_hifigan_tts(input=text, voice='zhizhe_emo')
wav = output[OutputKeys.OUTPUT_WAV]
with open('output.wav', 'wb') as f:
    f.write(wav)