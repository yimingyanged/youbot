<!--2579 words-->
<!--'level11_08_05_12' Time:0~460s Images:291-->
<!--'level07_17_04_12' Time:0~243s Images:450-->
<!--'level07_17_04_12' Time:380~570s Images:574-->
<!--'level07_17_04_12' Time:760~795s Images:598-->
<!--'level07_17_04_12' Time:990~992s Images:600-->
<launch>
	<node pkg="openfabmap2" type="learn_node" name="learn_node">
		<remap from="image" to="/stereo/left/image_raw"/>
		<param name="vocab" value="/media/psf/EVE/Datasets/Guiabot/codebook/vocab.yml"/>
		<param name="clTree" value="/media/psf/EVE/Datasets/Guiabot/codebook/clTree.yml"/>
		<param name="trainbows" value="/media/psf/EVE/Datasets/Guiabot/codebook/trainbows.yml"/>
		<param name="DetectorType" value="SURF"/>
		<param name="sampleRate" value="0.65"/>
		<param name="maxImages" value="600"/>
		<param name="clusterSize" value="0.5"/>
		<param name="visualise" value="true"/>
	</node>
</launch>