# self_driving

- To test run the controllers use below commands

Terminal 1: Start Carla simulator

```
cd /opt/carla-simulator/

./CarlaUE4.sh -prefernvidia
```

#### controller: PID

Terminal 2: To test the controller first navigate to the assignment folder
```
cd assignment_1

python3 scripts/main_control.py
```
After the testing is done completely then,

Terminal 3: To evaluate the controller performance first navigate to the assignment folder
```
cd assignment_1

python3 scripts/evaluate_controller_performance.py
```