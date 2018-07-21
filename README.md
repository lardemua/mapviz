# Mapviz

Aplicação desenvolvida por SwRI  para a qual foram desenvolvidos novos *plugins* adequados ao ATLASCAR2.

## Novos *Plugins*

   *Plugins* de origem podem ser observados no [Github](https://github.com/swri-robotics/mapviz)

```
    bestpos  -> representa a informação proveniente do tópico bestpos (GPS+INS).
    bestgnsspos  -> representa a informação proveniente do tópico bestgnsspos (GPS).
    global_planning -> representa informação relativa à navegação global. (tópicos /waypoints_*** mais informações no *package* global_planning ).
```




# Utilização - Launch file
```
<launch>
    <env name="ROSCONSOLE_FORMAT" value="[${thread}] [${node}/${function}:${line}]: ${message}"/>
    <node pkg="mapviz" type="mapviz" name="$(anon mapviz)" required="true" output="log"/>
    <node pkg="rosapi" type="rosapi_node" name="rosapi"/>
</launch>
```

```
roslaunch mapviz mapviz.launch
```


## Teste

```
Fixed frame: map
```

```
Target frame: base_link_imu
```

## Sistemas de eixos

World, map, base_link_imu  podem ser lançados da seguinte forma:


```
    <node pkg="swri_transform_util" type="imu_transform_publisher" name="imu_transform_publisher" output="screen"/>

  <node pkg="tf" type="static_transform_publisher" name="swri_transform" args="0 0 0 0 0 0 /world /map 100" />

    <node pkg="swri_transform_util" type="initialize_origin.py" name="initialize_origin" output="screen">
        <param name="local_xy_frame" value="/world"/>
        <param name="local_xy_origin" value="auto"/>
        <!-- "auto" setting will set the origin to the first gps fix that it recieves -->
        <remap from="gps" to="/gps/fix"/>
    </node>
```
Este código está inserido no ficheiro *novatel.launch*.



# Pedido de rota

Utilizar o *plugin*  point_click_publisher.
```
Topic: /destination
Frame: /wgs84
```

# Instalação

Através do github ATLASCAR.

Caso contrário, a componente de navegação global não poderá ser monitorizada. 

## Autor
* **Pedro Bouça Nova** - *Dissertação de Mestrado - 2018* -




