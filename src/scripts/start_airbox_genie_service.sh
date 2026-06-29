#!/usr/bin/env bash
set -euo pipefail

MODEL_NAME="${MODEL_NAME:-DeepSeek-R1-Distill-Qwen-7B}"
CONDA_ENV="${CONDA_ENV:-llm}"
CONDA_SH="${CONDA_SH:-/home/radxa/miniconda3/etc/profile.d/conda.sh}"
SAMPLES_DIR="${SAMPLES_DIR:-/home/radxa/ai-engine-direct-helper/samples}"
QAIRT_DIR="${QAIRT_DIR:-/home/radxa/qairt/2.37.1.250807}"
QAI_LIB_DIR="/home/radxa/miniconda3/envs/${CONDA_ENV}/lib/python3.12/site-packages/qai_appbuilder/libs"

source "${CONDA_SH}"
conda activate "${CONDA_ENV}"

export LD_LIBRARY_PATH="${QAIRT_DIR}/lib/aarch64-oe-linux-gcc11.2:${QAI_LIB_DIR}:${LD_LIBRARY_PATH:-}"
export ADSP_LIBRARY_PATH="${QAIRT_DIR}/lib/hexagon-v73/unsigned"

cd "${SAMPLES_DIR}"
exec python genie/python/GenieAPIService.py --modelname "${MODEL_NAME}" --loadmodel --profile "$@"
