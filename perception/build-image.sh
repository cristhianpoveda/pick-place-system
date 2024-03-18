#!/bin/bash
REBUILD=0
function usage {
  printf "Usage: bash build-image.sh [OPTIONS] \n"
  printf "Options: \n"
  printf " -h  Display this help message.\n"
  printf " -i  Base image name \n"
  printf " -t  Base image tag \n"
  printf " -n  New image tag \n"
  exit 0
}
while getopts :i:t:n:h opt; do
  case $opt in
    h) usage ;;
    i) BASE_IMAGE=${OPTARG};;
    t) BASE_TAG=${OPTARG};;
    n) TAG=${OPTARG};;
    r) REBUILD=1 ;;
    *) printf "build image: "$1" is not a valid option. \n"
       usage
      exit 1
    esac
done

docker pull ${BASE_IMAGE}:${BASE_TAG}
dirname=${PWD}
# NAME="${dirname%"${dirname##*[!/]}"}"
# NAME="${NAME##*/}"
# NAME=${NAME:-/}
UIDD="$(id -u $USER)"
GIDD="$(id -g $USER)" 
if [ "$REBUILD" -eq 1 ]; then
  docker build \
  --no-cache \
  --build-arg BASE_IMAGE=${BASE_IMAGE} \
  --build-arg BASE_TAG=${BASE_TAG} \
  --build-arg UID=${UIDD} \
  --build-arg GID=${GIDD} \
  -t ${TAG} .
else
  docker build \
  --build-arg BASE_IMAGE=${BASE_IMAGE} \
  --build-arg BASE_TAG=${BASE_TAG} \
  --build-arg UID=${UIDD} \
  --build-arg GID=${GIDD} \
  -t ${TAG} .
fi