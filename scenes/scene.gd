extends Spatial



func _on_Timer_timeout():
	add_child(preload("projectile.tscn").instance())
	$Timer.start()
