/* http://css-tricks.com/snippets/javascript/showhide-element/ */

function toggle_visibility(id) {
	var e = document.getElementById(id);
	
	if(e.style.display == 'block')
		e.style.display = 'none';
	else
		e.style.display = 'block';
}



