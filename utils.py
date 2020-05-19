import  numpy as np
from io import BytesIO

def ndarray_to_bytes(nda: np.ndarray) -> bytes:
    nda_bytes = BytesIO()
    np.save(nda_bytes, nda, allow_pickle=False)
    return nda_bytes.getvalue()


def bytes_to_ndarray(nda_proto: bytes) -> np.ndarray:
    nda_bytes = BytesIO(nda_proto)
    return np.load(nda_bytes, allow_pickle=False)
