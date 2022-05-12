# Reneses CK RA6M5 Firmware

## Build firmware using Docker

1. Build container

    ```
    docker build -t edge-impulse-renesas .
    ```

1. Build firmware

    ```
    docker run --rm -v $PWD:/app/workspace/firmware-renesas-ck-ra6m5 edge-impulse-renesas
    ```
