import torch
import numpy as np
import onnxruntime
from pathlib import Path
import onnxruntime as ort

# 模型相对路径
model_relative_path = '../resource/silero_vad.onnx'
current_dir = Path(__file__).parent
model_path = current_dir / model_relative_path

class VoiceActivityDetection():

    def __init__(self,model_path):
        print("loading session")
        if not Path(model_path).exists():
            raise FileNotFoundError(f"VAD模型未找到: {model_path}")
        opts = ort.SessionOptions()
        opts.log_severity_level = 3
        opts.inter_op_num_threads = 1
        opts.intra_op_num_threads = 1
        print("loading onnx model")
        self.session = ort.InferenceSession(str(model_path), providers=['CPUExecutionProvider'],
                                                    sess_options=opts)
        print("reset states")
        self.reset_states()
        self.sample_rates = 16000

    def _validate_input(self, x, sr: int):
        """
        Validate and reshape input audio to ensure it's 2D (batch, samples).
        """
        if x.ndim == 1:
            x = np.expand_dims(x, axis=0)
        if x.ndim > 2:
            raise ValueError(f"Too many dimensions for input audio chunk {x.ndim}")
        if sr / x.shape[1] > 31.25:
            raise ValueError("Input audio chunk is too short")

        return x, sr

    def reset_states(self, batch_size=1):
        self._h = np.zeros((2, batch_size, 64)).astype('float32')
        self._c = np.zeros((2, batch_size, 64)).astype('float32')
        self._last_sr = 0
        self._last_batch_size = 0

    def __call__(self, x, sr: int):
        x, sr = self._validate_input(x, sr)
        batch_size = x.shape[0]

        if not self._last_batch_size:
            self.reset_states(batch_size)
        if (self._last_sr) and (self._last_sr != sr):
            self.reset_states(batch_size)
        if (self._last_batch_size) and (self._last_batch_size != batch_size):
            self.reset_states(batch_size)

        ort_inputs = {'input': x.astype(np.float32), 'h': self._h, 'c': self._c, 'sr': np.array(sr, dtype='int64')}
        ort_outs = self.session.run(None, ort_inputs)
        out, self._h, self._c = ort_outs

        self._last_sr = sr
        self._last_batch_size = batch_size

        out = torch.tensor(out)
        return out

    def audio_forward(self, x, sr: int, num_samples: int = 512):
        outs = []
        x, sr = self._validate_input(x, sr)

        if x.shape[1] % num_samples:
            pad_num = num_samples - (x.shape[1] % num_samples)
            x = torch.nn.functional.pad(x, (0, pad_num), 'constant', value=0.0)

        self.reset_states(x.shape[0])
        for i in range(0, x.shape[1], num_samples):
            wavs_batch = x[:, i:i + num_samples]
            out_chunk = self.__call__(wavs_batch, sr)
            outs.append(out_chunk)

        stacked = torch.cat(outs, dim=1)
        return stacked.cpu()
