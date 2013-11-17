import wordmaker_python as libwordmaker

wm = libwordmaker.WordMaker()

vocabPath = "../../Examples/Resources/Vocabularies/OxfordVocab_Surf_11k/"
vocabName = "OxfordVocab_Surf_11k"
descriptor_dim = 128

wm.LoadVocabularyAndSetup(vocabPath,vocabName,descriptor_dim)

ImagePath = '../../Examples/Resources/Sample_Data/Images/00001.jpg'

words = wm.GetBagOfWordsForImage(ImagePath)
print words

#Also exposed are
#wm.SetSURFThreshold(25.0)
#bUpright = true
#bSurf128 = true
#wm.SetSURFOptions(bUpright,bSurf128)

#To more info about a method, check the doc string
#print wm.GetBagOfWordsForImage.__doc__
