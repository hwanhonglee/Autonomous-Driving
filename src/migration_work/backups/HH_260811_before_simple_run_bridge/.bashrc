# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

# Created by `pipx` on 2024-10-30 07:50:24
export PATH="$PATH:/home/a/.local/bin"
export CCACHE_DIR="/var/tmp/ccache"
export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"

# source
# ROS source
source /opt/ros/humble/setup.bash
# Autoware source
source ~/autoware/install/setup.bash
# Camera source

# export
# Cuda & Autoware universe domain
export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"

# ROS 2 
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=10
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
if [ ! -e /tmp/cycloneDDS_configured ]; then
    sudo sysctl -w net.core.rmem_max=2147483647
    sudo sysctl -w net.ipv4.ipfrag_time=3
    sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728 # (128 MB)
    sudo ip link set lo multicast on
    touch /tmp/cycloneDDS_configured
fi

# ROS 2 Middle Ware
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# HH_260810 - Bound CycloneDDS to PC1's 192.168.9.2 vehicle-PC NIC so Domain 10 can discover PC2 and PC3 deterministically.
export CYCLONEDDS_URI=file:///home/a/autoware/src/migration_work/config/cyclonedds_pc1.xml

# alias
# build
alias build_autoware='cd ~/autoware && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
alias build_autoware_part='cd ~/autoware && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select'

# run
# HH_260810 - Added a duplicate-start and actuator-path guard while preserving the existing run_autoware alias.
alias run_autoware='/home/a/autoware/src/migration_work/scripts/run_autoware_safe.sh'
# HH_260810 - Preserved the planning-simulator alias as a Domain 110 internal-control test with engage and physical actuation disabled.
alias run_planning_universe='/home/a/autoware/src/migration_work/scripts/run_planning_universe_safe.sh'
# HH_260811 - Restored run_bridge physical CAN transmit and the control adapter for supervised control verification while retaining the guarded alias.
alias run_bridge='/home/a/autoware/src/migration_work/scripts/run_bridge_safe.sh'
alias plotjuggler='ros2 run plotjuggler plotjuggler'

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/a/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/a/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/home/a/anaconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/a/anaconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<
