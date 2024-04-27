#!/usr/bin/env bash

LOGGER_DATA_FORMAT=${LOG_DATA_FORMAT:-'%Y/%m/%d %H:%M:%S'}
LOGGER_LEVEL=${LOGGER_LEVEL:-1} # 0: debug, 1: info, 2: notice, 3: warning, 4: error
LOGGER_SAVE_FILENAME=${LOGGER_SAVE_FILENAME:-""}
# color {{{
LOGGER_DEBUG_COLOR=${LOGGER_DEBUG_COLOR:-"\\033[3m"}
LOGGER_INFO_COLOR=${LOGGER_INFO_COLOR:-"\\033[32m"}
LOGGER_NOTICE_COLOR=${LOGGER_NOTICE_COLOR:-"\\033[36m"}
LOGGER_WARNING_COLOR=${LOGGER_WARNING_COLOR:-"\\033[33m"}
LOGGER_ERROR_COLOR=${LOGGER_ERROR_COLOR:-"\\033[31m"}
LOGGER_RESET_COLOR="\\033[0m"
LOGGER_TIME_COLOR=${LOGGER_TIME_COLOR:-"\\033[30m"}
LOGGER_FILENAME_COLOR=${LOGGER_FILENAME_COLOR:-"\\033[35m"}
LOGGER_COLORS=("$LOGGER_DEBUG_COLOR" "$LOGGER_INFO_COLOR" "$LOGGER_NOTICE_COLOR" "$LOGGER_WARNING_COLOR" "$LOGGER_ERROR_COLOR")
# }}}
if [ "${LOGGER_LEVELS:-}" = "" ];then
  LOGGER_LEVELS=("DEBUG" "INFO" "NOTICE" "WARNING" "ERROR")
fi

# Function {{{

# @param  level: int
# @return level string
_logger_level() {
  [ $# -eq 1 ] || return
  local level=$1
  printf "${LOGGER_LEVELS[${level}]}"
}

_logger_time() {
  printf "$(date +"${LOGGER_DATA_FORMAT}")"
}

_logger_file() {
  local len=${#BASH_SOURCE[@]}
  printf "$(basename ${BASH_SOURCE[((len - 1))]})"
}

_logger() {
  [ $# -ge 2 ] || return
  local level="$1"
  shift
  [ ${level} -ge ${LOGGER_LEVEL} ] || return
  
  local level_prefix="${LOGGER_COLORS[${level}]}[$(_logger_level ${level})]"
  local time_prefix="${LOGGER_TIME_COLOR}$(_logger_time)"
  local filename_prefix="${LOGGER_FILENAME_COLOR}($(_logger_file))"
  local msg_prefix="${level_prefix}${time_prefix}${filename_prefix}"
  local msg="${msg_prefix} \\033[${LOGGER_COLORS[${level}]}$@ ${LOGGER_RESET_COLOR}"
  printf "${msg}\n"
  if [ ${LOGGER_SAVE_FILENAME} ]; then
    msg_prefix="[$(_logger_level ${level})]$(_logger_time)($(_logger_file))"
    msg="${msg_prefix} "$@""
    printf "${msg}\n" >> ${LOGGER_SAVE_FILENAME}
  fi
}

debug() {
  _logger 0 "$@"
}

info() {
  _logger 1 "$@"
}

notice() {
  _logger 2 "$@"
}

warn() {
  _logger 3 "$@"
}

error() {
  _logger 4 "$@"
}

close_log() {
  if [ ${LOGGER_SAVE_FILENAME} ]; then
    echo "----------------------" >> ${LOGGER_SAVE_FILENAME}
  fi
} 
# }}}
