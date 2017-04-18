# ras_mapping
A package for mapping the enviroment.

## map_builder
Listens to `/scan`  and publishes a pointCloud2 msg on `/mapping/map` correlating to the `N` latest scans. 
`N` needs to be set as a parameter.
