package link_errors

type LinkDoesNotExistError struct{}

func (e *LinkDoesNotExistError) Error() string {
	return "Link does not exist"
}
