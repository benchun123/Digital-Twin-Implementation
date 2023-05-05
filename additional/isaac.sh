#!/bin/bash
# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/benchun/Software/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/benchun/Software/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/home/benchun/Software/anaconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/benchun/Software/anaconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<


conda activate isaac-sim
pwd | xclip -sel clip
cd ~/.local/share/ov/pkg/isaac_sim-2022.1.1/
source setup_conda_env.sh
cd "$(xclip -sel clip -o)"
