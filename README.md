# TP Final — Creation de driver

1. Cloner ce dossier dans votre `app_worksapce`
2. Completez `dummy_driver` et faites votre PR
3. Completez le driver BME 280 avec la [documentation](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/#technical) et faites votre PR



COMPTE RENDU Dylan GIMENEZ Gweltaz BURLOT

La première étape à été de modifier le fichier .dts ainsi que Kconfig. Il a fallut rajouter la partie "include" les "properties" et la partie "compatible". Ces parties permettent de faire le lien entre la configuration matériel et logicielle du driver via une variable crée à la compilation [DT_HAS_<VENDOR>_<DEVICE>_ENABLED].
L'app overlay permet de définir les valeurs par defaut décrites dans la partie properties du .dts .

Il est encore difficile de déterminer exactement ce qui est optionnel de ce qui est obligatoir, la syntaxe est bien définie sur les slides mais nous trouvons que cela manque de contexte malgré les quelques exercices précedemment faits. 

Programmation du Driver fummy
LA premiere fonction du .C est "my_sensor_channel_supported()". 
Nous pensons qu'il s'agit d'une fonction permettant de déterminer si le "channel" utilisé (qui représente la nature de la mesure, ici une température). Donc si le channel vaut SENSOR_CHAN_AMBIENT_TEMP alors on renvoie True.