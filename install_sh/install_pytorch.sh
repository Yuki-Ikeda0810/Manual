#!/bin/bash
# 参考①：https://github.com/pytorch/pytorch
# 参考②：https://github.com/pytorch/vision
# 参考③：https://github.com/pytorch/audio
# 参考④：https://pytorch.org/get-started/previous-versions/

PYTORCH_LIST=("1.13.1" "1.13.0" "1.12.0" "1.11.0" "1.10.0"  "1.9.1"  "1.8.2"  "1.7.1")
PTVSION_LIST=("0.14.1" "0.14.0" "0.13.0" "0.12.0" "0.11.0"  "0.10.1" "0.9.2"  "0.8.2")
PTAUDIO_LIST=("0.13.1" "0.13.0" "0.12.0" "0.11.0" "0.10.0"  "0.9.1"  "0.8.2"  "0.7.2")
CUDA_LIST=(   "cu117"  "cu117"  "cu116"  "cu113"  "cu111"   "cu111"  "cu113"  "110")

PY_NO="TBD"
PYTORCH_VER=$1
PYTORCH_VER=${PYTORCH_VER//-}


# Check if the PyTorch version to install exists (or if supported)
if [[ "${PYTORCH_LIST[*]}" =~ (^|[[:space:]])"${PYTORCH_VER}"($|[[:space:]]) ]]
then
    # Check the number of PyTorch version in the list
    cnt=0
    for i in "${PYTORCH_LIST[@]}"
    do
        PY_NO=$cnt

        if [[ " ${i} " == " ${PYTORCH_VER} " ]]
        then
            break
        fi
        let cnt++
    done
    echo "╔══╣ Install: PyTorch $PYTORCH_VER (STARTING) ╠══╗"
else
    echo "Please select one of the following PyTorch versions"
    echo ${PYTORCH_LIST[*]}
    echo "For example: $ bash install_pytorch.sh --1.12.0"
    exit
fi

# python3 -m pip install \
#     numpy \
#     matplotlib \
#     tqdm

if [[ "${PYTORCH_LIST[PY_NO]}" < "1.11.0" ]]; then
    python3 -m pip install \
        torch==${PYTORCH_LIST[PY_NO]}+${CUDA_LIST[PY_NO]} \
        torchvision==${PTVSION_LIST[PY_NO]}+${CUDA_LIST[PY_NO]} \
        torchaudio==${PTAUDIO_LIST[PY_NO]} \
        -f https://download.pytorch.org/whl/torch_stable.html

else
    python3 -m pip install \
        torch==${PYTORCH_LIST[PY_NO]}+${CUDA_LIST[PY_NO]} \
        torchvision==${PTVSION_LIST[PY_NO]}+${CUDA_LIST[PY_NO]} \
        torchaudio==${PTAUDIO_LIST[PY_NO]} \
        --extra-index-url https://download.pytorch.org/whl/${CUDA_LIST[PY_NO]}
fi

echo "╚══╣ Install: PyTorch $PYTORCH_VER (FINISHED) ╠══╝"
