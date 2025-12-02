# Go2-W-Praxisphase
Implementation und Inbetriebname Go2-W

Zum starten der Unitree SDK ("https://github.com/unitreerobotics/unitree_sdk2/tree/2.0.2") auf dem Jetson muss die IP des Jetson boards auf 192.168.123.19 gesetzt werden. Die MCU nutzt die 192.168.123.18, was bedeutet dass die IP doppelt besetzt ist wenn der Jetson wie von Unitree beschrieben auch auf 192.168.123.18 gesetzt wird. Beim Funktionsaufruf aus der SDK kommt es zu Problemen (es hat in unserem Fall einfach nicht funktioniert) wenn die IP addressen übereinstimmen. Nach ändern der IP addresse konnten wir die SDK lokal auf dem Roboter betreiben.
