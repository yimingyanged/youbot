import fabmap_python as fabmap

# Configure a FabMap object
vocabPath = "../../Examples/Resources/Vocabularies/OxfordVocab_Surf_11k/"
vocabName = "OxfordVocab_Surf_11k"
p_observe_given_exists = 0.39
p_observe_given_not_exists = 0.0
p_at_new_place = 0.9
VocabSize = fabmap.DetermineVocabSize(vocabPath,vocabName)
LikelihoodSmoothingFactor = 0.99

fm = fabmap.FabMapCalculator(vocabPath,vocabName,
			      p_observe_given_exists,p_observe_given_not_exists,
			      p_at_new_place,VocabSize,LikelihoodSmoothingFactor)

# Supply a .moos configuration file.
fm.ConfigureForExternalCalls("../../Examples/RunningFabMapExample/bin/FabMap_BatchConfig.moos")

# Try some sample calls
observed_words = [1,2,3]  # Dummy bag-of-words data.
                          # See WordMaker/python/wordmaker_example.py for an
			  # example of how to extract a bag-of-words from a real image.
pdf = fm.ProcessObservation(observed_words)
fm.ConfirmLastMatch()
observed_words = [7,8,9]
pdf = fm.ProcessObservation(observed_words)
print pdf

#To get more info about a method, check the doc string
#print wm.ProcessObservation.__doc__

#Also exposed
#fm.AddLocationToMap
#fm.DiscardInfoFromLastMatch
#fm.ConfirmLastMatch
#fm.Reset

#And some other APIs for fine-grained external control (should not be necessary by most users):
#
#fm.ProcessObservation_ComputeLikelihoodOnly
#fm.ComputeMostLikelyMatch  - A convenience version of ProcessObservation_ComputeLikelihoodOnly which only returns the ID of the place with the highest raw likelihood
#fm.AddLocationToMap	    - Directly add a new location to the map, bypassing normal channels. Mixing this call with the other APIs above is not recommende. E.g. the motion prior will no longer be correctly defined.

