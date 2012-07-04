Die nodes/pipefollowing_server.py startet einen ActionServer und noch nicht das Verhalten selber. Die Node sollte dauerhaft laufen und wird am besten über das Launchfile gestartet.

Das Verhalten kann dann per actionlib gestartet/gestoppt werden (PipeFollowingAction).
Zum Testen kann nodes/start_action.py dazu verwendet werden.
