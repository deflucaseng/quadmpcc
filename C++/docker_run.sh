#!/bin/bash
docker run -it --rm -p 8080:8080 -v $SSH_AUTH_SOCK:/ssh-agent -e SSH_AUTH_SOCK=/ssh-agent -v "$(pwd):/app/MPCC" mpcc
