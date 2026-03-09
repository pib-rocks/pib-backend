#!/bin/bash
(cd /home/pib/app/pib-backend && sudo docker compose --profile "*" up --remove-orphans) &
(cd /home/pib/app/cerebra && sudo docker compose --profile "*" up --remove-orphans ) &
wait
