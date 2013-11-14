<!-- trainOut_22_05_12 && trainOut2_22_05_12 -->
<launch>
	<node pkg="openfabmap2" type="learn_node" name="learn_node">
		<remap from="image" to="/stereo/left/image_raw"/>
		<param name="vocab" value="/media/psf/EVE/Datasets/Guiabot/codebook/R02/vocab.yml"/>
		<param name="clTree" value="/media/psf/EVE/Datasets/Guiabot/codebook/R02/clTree.yml"/>
		<param name="trainbows" value="/media/psf/EVE/Datasets/Guiabot/codebook/R02/trainbows.yml"/>
		<param name="DetectorType" value="SURF"/>
		<param name="sampleRate" value="0.65"/>
		<param name="maxImages" value="300"/>
		<param name="clusterSize" value="0.5"/>
		<param name="visualise" value="true"/>
	</node>
</launch>
