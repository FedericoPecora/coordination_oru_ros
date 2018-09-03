^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package coordination_oru_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2018-09-03)
------------------
* post demo
* Arena demo done
* New cycle 3 and reverse service implemented in tracker
* Added remote launch files and modified truncating service
* Added arena launch files; now depends on coordination_oru v. 0.3.2
* Bumped up version of coordination_oru to 0.3.1
* Point and click remote launch.
* Fixed remapping of topics...
* Added service to abort mission
* Adding ability to interrupt mission (incomplete); now estimates vel and dist from RobotReport message
* Using refactored mission stack management in coordination_oru (new release 0.2.0)
* Compute tasks sequentially in marshalling lane example
* Now depends on new coordination_oru release (0.1.9)
* Added new launch files
* Added deadlock elimination with prioritized motion planning
* Added launch file for smp planner
* Fixed two trucks in robot lab launch file
* Added preliminary version of RobotLab coord demo
* Added multiple trucks launch file in robot_lab
* Contributors: Federico Pecora, Henrik Andreasson

0.4.0 (2018-05-15)
------------------
* Corrected iliad_smp_global_planner reference in launch files
* Merge pull request `#5 <https://github.com/FedericoPecora/coordination_oru_ros/issues/5>`_ from FedericoPecora/message_generation
  Refactoring into separate packages for message dependencies
* added launch install
* Contributors: Federico Pecora, Marc Hanheide

0.3.1 (2018-05-14)
------------------
* added missing dep
* Contributors: Marc Hanheide

0.3.0 (2018-05-14)
------------------
* faithful attempt...
* make messages an own package
* Contributors: Marc Hanheide

0.2.2 (2018-05-14)
------------------

0.2.1 (2018-05-14)
------------------
* .gitignore updated to not include cradle build files
* message generation
* added message_generation
* Contributors: Marc Hanheide

0.2.0 (2018-05-14)
------------------
* Merge branch 'master' of github.com:FedericoPecora/coordination_oru_ros
* changed install to include dependency packages
* Changed to RVizVisualization in NCFM main node
* Added RVIZVisualization in MainNode
* Updated to new version of coordination_oru (0.1.4)
* Now depends on snapshots of coordination_oru and metacsp
* Contributors: Federico Pecora, Marc Hanheide

* Merge branch 'master' of github.com:FedericoPecora/coordination_oru_ros
* changed install to include dependency packages
* Changed to RVizVisualization in NCFM main node
* Added RVIZVisualization in MainNode
* Updated to new version of coordination_oru (0.1.4)
* Now depends on snapshots of coordination_oru and metacsp
* Contributors: Federico Pecora, Marc Hanheide
