extends Spatial



func _on_Timer_timeout():
	var tmp=preload("projectile.tscn").instance()
	tmp.translation.y+=1
	add_child(tmp)
	$Timer.start()
