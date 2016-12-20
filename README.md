This work is a part of **On Attitude Estimation with Smartphones** project  
http://tyrex.inria.fr/mobile/benchmarks-attitude/

Abstract
--------

We investigate the precision of attitude estimation algorithms in the particular context of pedestrian navigation with commodity smartphones and their inertial/magnetic sensors. We report on an extensive comparison and experimental analysis of existing algorithms. We focus on typical motions of smartphones when carried by pedestrians. We use a precise ground truth obtained from a motion capture system. We test state-of-the-art attitude estimation techniques with several smartphones, in the presence of magnetic perturbations typically found in buildings. We discuss the obtained results, analyze advantages and limits of current technologies for attitude estimation in this context. Furthermore, we propose a new technique for limiting the impact of magnetic perturbations with any attitude estimation algorithm used in this context. We show how our technique compares and improves over previous works.



How it works ?
--------------

To start, open a matlab console. Go to **src** folder. Add this folder and subfolders to the matlab path:

    cd YOUR_PATH/benchmarks-attitude-smartphones/src/
    loadProject;

Create a new benchmark instance. Load sensors noises, calibrations and datasets. Then start the benchmark process:

    benchmark = AttitudeBenchmarks;
    benchmark.load;
	benchmark.process;

The benchmark process takes a while and memory if you compute all datasets with all algorithms, all sampling, all calibrations... (e.g. 26 hours on a macbook pro and 25 Go). In order to avoid to restart the full process each time, you can save/open the benchmark:
	
	save(AttitudeBenchmarks.url, 'benchmark', '-v7.3'); % You may need to create saved folder in the root directory
	load(AttitudeBenchmarks.url);

Results are stored in **AttitudeBenchmarkResults** class and aggregated data on means can be computed using **compare** function

	results = benchmark.results;
	results.compare('calibration');
	results.compare('motions');
	results.compare('motionsmag');
	results.compare('mag');
	results.compare('devices');
	results.compare('sampling');
	results.compare('processingTime');


Some stats on datasets can be computed:

	benchmark.stats;


License
-------

This project is under the [CeCILL](http://www.cecill.info/index.en.html) license.

Authors
-------

Thibaud Michel  
<thibaud.michel@inria.fr>  

Hassen Fourati  
Pierre Genev&egrave;s  
Nabil Laya&iuml;da  

[Tyrex Team](http://tyrex.inria.fr), LIG, Inria (France), 2016  
[NEcS Team](http://necs.inria.fr), Gipsa-Lab, Inria (France), 2016
