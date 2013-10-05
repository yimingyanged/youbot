youbot_edinburgh
================


Making changes to a submodule
--------------

Here we want to first push our changes to the submodule’s upstream repo, and then record the change in the parent project repo. It’s very important to not skip the first part, as that would break the submodule for other developers when they pull changes to the parent project with an updated reference to a nonexistent state of the submodule repo.

	$ cd <submodule path>
	// do stuff
	$ git commit ... // Changes committed to submodule; parent repo only recognizes that submodule's commit has changed
	$ git push ... // Push submodule changes; parent repo unaffected
	$ cd <anywhere within parent repo>
	$ git commit ... // Commit updated reference to new submodule state (reference by commit)
	$ git push ...
	// Tell your fellow coders to be sure and update submodules when they next pull.

Cloning a repo with submodules for the first time
--------------

After cloning the repo, initialize and update your submodules. git submodule init sets up the repo structure. git submodule update populates submodule files by pulling their commits.

	$ git clone ...
	$ git submodule init
	$ git submodule update

Pulling commits including updates to submodules
--------------

	$ git pull ...
	$ git submodule update
