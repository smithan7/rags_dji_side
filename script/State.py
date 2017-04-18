class State(object):
	def __init__( self, arg ):
		self.x = arg[0]
		self.y = arg[1]
		self.f = float("inf")
		self.g = float("inf")
