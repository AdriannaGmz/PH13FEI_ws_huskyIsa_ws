time timestamp		#Timestamp of the first sample
uint32 seq		#Counter
float32 sampleRate	#Estimated current sample rate
uint8 length		#Length of data for a single channel
float32[] samples	#Array of all samples. Format: sample1_ch1, sample1_ch2, sample1_chN, sample2_ch1, sample2_ch2, sample2_chN ... sampleM_chN
